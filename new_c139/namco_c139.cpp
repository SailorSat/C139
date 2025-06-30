// license:BSD-3-Clause
// copyright-holders:Angelo Salese, John Bennett, Ariane Fugmann
/***************************************************************************

    Namco C139 - Serial I/F Controller

    (from assault schematics, page 5-18 and 5-19)
    connected to M5M5179P RAM with a 13-bit address bus, and 9 bit data bus
    connected to host cpu with a 14*-bit address bus, and 13 bit data bus
    2 clock inputs - 16M and 12M
    uses 9-N-1 encoding, with 1 or 2 mbits

    NOTES:
    - C422 seems to be a pin compatible upgrade to C139, probably supporting higher clock speeds?

***************************************************************************/

#include "emu.h"
#include "namco_c139.h"

#include "emuopts.h"
#include "multibyte.h"

#include "asio.h"

#include <iostream>

#define VERBOSE 0
#include "logmacro.h"


class namco_c139_device::context
{
public:
	context(namco_c139_device &device) :
		m_device(device),
		m_acceptor(m_ioctx),
		m_sock_rx(m_ioctx),
		m_sock_tx(m_ioctx),
		m_timeout_tx(m_ioctx),
		m_stopping(false),
		m_forward(false),
		m_state_rx(0U),
		m_state_tx(0U)
	{
	}

	void start()
	{
		m_thread = std::thread(
			[this]()
			{
				LOG("C139: network thread started\n");
				try {
					m_ioctx.run();
				} catch (const std::exception& e) {
					LOG("C139: Exception in network thread: %s\n", e.what());
				} catch (...) { // Catch any other unknown exceptions
					LOG("C139: Unknown exception in network thread\n");
				}
				LOG("C139: network thread completed\n");
			});
	}

	void reset(std::string localhost, std::string localport, std::string remotehost, std::string remoteport, bool forward)
	{
		m_ioctx.post(
			[this, localhost = std::move(localhost), localport = std::move(localport), remotehost = std::move(remotehost), remoteport = std::move(remoteport), forward = std::move(forward)] ()
			{
				std::error_code err;
				asio::ip::tcp::resolver resolver(m_ioctx);

				for (auto&& resolveIte : resolver.resolve(localhost, localport, asio::ip::tcp::resolver::flags::address_configured, err))
				{
					m_localaddr = resolveIte.endpoint();
					LOG("C139: localhost = %s\n", *m_localaddr);
				}
				if (err)
				{
					LOG("C139: localhost resolve error: %s\n", err.message());
				}

				for (auto&& resolveIte : resolver.resolve(remotehost, remoteport, asio::ip::tcp::resolver::flags::address_configured, err))
				{
					m_remoteaddr = resolveIte.endpoint();
					LOG("C139: remotehost = %s\n", *m_remoteaddr);
				}
				if (err)
				{
					LOG("C139: remotehost resolve error: %s\n", err.message());
				}

				m_forward = forward;
				if (m_acceptor.is_open())
					m_acceptor.close(err);
				if (m_sock_rx.is_open())
					m_sock_rx.close(err);
				if (m_sock_tx.is_open())
					m_sock_tx.close(err);
				m_timeout_tx.cancel();
				m_state_rx.store(0);
				m_state_tx.store(0);
				start_accept();
				start_connect();
			});
	}

	void stop()
	{
		m_ioctx.post(
			[this]()
			{
				m_stopping = true;
				std::error_code err;
				if (m_acceptor.is_open())
					m_acceptor.close(err);
				if (m_sock_rx.is_open())
					m_sock_rx.close(err);
				if (m_sock_tx.is_open())
					m_sock_tx.close(err);
				m_timeout_tx.cancel();
				m_state_rx.store(0);
				m_state_tx.store(0);
				m_ioctx.stop();
			});
		m_work_guard.reset();
		if (m_thread.joinable()) {
			m_thread.join();
		}
	}

	void check_sockets()
	{
	}

	bool connected()
	{
		return m_state_rx.load() == 2 && m_state_tx.load() == 2;
	}

	unsigned receive(uint8_t* buffer, unsigned data_size)
	{
		if (m_state_rx.load() < 2)
			return UINT_MAX;

		if (data_size > m_fifo_rx.used())
			return 0;

		return m_fifo_rx.read(&buffer[0], data_size, false);
	}

	unsigned send(uint8_t* buffer, unsigned data_size)
	{
		if (m_state_tx.load() < 2)
			return UINT_MAX;

		if (data_size > m_fifo_tx.free())
		{
			LOG("C139: TX buffer overflow\n");
			return UINT_MAX;
		}

		bool const sending = m_fifo_tx.used();
		m_fifo_tx.write(&buffer[0], data_size);
		if (!sending)
			m_ioctx.post(
				[this]()
				{
					start_send_tx();
				});
		return data_size;
	}

private:
	class fifo
	{
	public:
		fifo() :
			m_wp(0),
			m_rp(0)
		{
		}

		unsigned write(uint8_t* buffer, unsigned data_size)
		{
			unsigned current_rp = m_rp.load(std::memory_order_acquire);
			unsigned current_wp = m_wp.load(std::memory_order_relaxed);

			// calculate free space
			unsigned data_free = (BUFFER_SIZE + current_rp - current_wp - 1) % BUFFER_SIZE;

			// sanity checks
			if (data_size > data_free)
				return UINT_MAX;
			if (data_size == 0)
				return 0;

			unsigned data_used = 0;

			// first part (up to end)
			unsigned block = std::min(data_size, BUFFER_SIZE - current_wp);
			std::copy_n(&buffer[0], block, &m_buffer[current_wp]);
			data_used += block;
			current_wp = (current_wp + block) % BUFFER_SIZE;

			// second part (from beginning, if wrapped)
			if (data_used < data_size)
			{
				block = data_size - data_used;
				std::copy_n(&buffer[data_used], block, &m_buffer[current_wp]);
				data_used += block;
				current_wp += block;
			}

			m_wp.store(current_wp, std::memory_order_release);
			return data_used;
		}

		unsigned read(uint8_t* buffer, unsigned data_size, bool peek)
		{
			unsigned current_wp = m_wp.load(std::memory_order_acquire);
			unsigned current_rp = m_rp.load(std::memory_order_relaxed);

			// calculate available data
			unsigned data_avail = (BUFFER_SIZE + current_wp - current_rp) % BUFFER_SIZE;

			// sanity checks
			if (data_size > data_avail)
				return UINT_MAX;
			if (data_size == 0)
				return 0;

			unsigned data_used = 0;

			// first part (up to end)
			unsigned block = std::min(data_size, BUFFER_SIZE - current_rp);
			std::copy_n(&m_buffer[current_rp], block, &buffer[0]);
			data_used += block;
			current_rp = (current_rp + data_used) % BUFFER_SIZE;

			// second part (from beginning, if wrapped)
			if (data_used < data_size)
			{
				block = data_size - data_used;
				std::copy_n(&m_buffer[current_rp], block, &buffer[data_used]);
				data_used += block;
				current_rp += block;
			}
			if (!peek)
			{
				m_rp.store(current_rp, std::memory_order_release);
			}
			return data_used;
		}

		void consume(unsigned data_size)
		{
			unsigned current_wp = m_wp.load(std::memory_order_acquire);
			unsigned current_rp = m_rp.load(std::memory_order_relaxed);

			//  available data
			unsigned data_avail = (BUFFER_SIZE + current_wp - current_rp) % BUFFER_SIZE;

			// sanity check
			if (data_size > data_avail)
				data_size = data_avail;

			current_rp = (current_rp + data_size) % BUFFER_SIZE;
			m_rp.store(current_rp, std::memory_order_release);
		}

		unsigned used()
		{
			unsigned current_wp = m_wp.load(std::memory_order_acquire);
			unsigned current_rp = m_rp.load(std::memory_order_acquire);
			return (BUFFER_SIZE + current_wp - current_rp) % BUFFER_SIZE;
		}

		unsigned free()
		{
			unsigned current_wp = m_wp.load(std::memory_order_acquire);
			unsigned current_rp = m_rp.load(std::memory_order_acquire);
			return (BUFFER_SIZE + current_rp - current_wp - 1 + BUFFER_SIZE) % BUFFER_SIZE;
		}

		void clear()
		{
			m_wp.store(0, std::memory_order_release);
			m_rp.store(0, std::memory_order_release);
		}

	private:
		static constexpr unsigned BUFFER_SIZE = 0x80000;
		std::atomic<unsigned> m_wp;
		std::atomic<unsigned> m_rp;
		std::array<uint8_t, BUFFER_SIZE> m_buffer;
	};

	void start_accept()
	{
		if (m_stopping)
			return;

		std::error_code err;
		m_acceptor.open(m_localaddr->protocol(), err);
		m_acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true));
		if (!err)
		{
			m_acceptor.bind(*m_localaddr, err);
			if (!err)
			{
				m_acceptor.listen(1, err);
				if (!err)
				{
					osd_printf_verbose("C139: RX listen on %s\n", *m_localaddr);
					m_acceptor.async_accept(
						[this](std::error_code const& err, asio::ip::tcp::socket sock)
						{
							if (err)
							{
								LOG("C139: RX error accepting - %d %s\n", err.value(), err.message());
								std::error_code e;
								m_acceptor.close(e);
								m_state_rx.store(0);
								start_accept();
							}
							else
							{
								LOG("C139: RX connection from %s\n", sock.remote_endpoint());
								std::error_code e;
								m_acceptor.close(e);
								m_sock_rx = std::move(sock);
								m_sock_rx.set_option(asio::socket_base::keep_alive(true));
								m_state_rx.store(2);
								start_receive_rx();
							}
						});
					m_state_rx.store(1);
				}
			}
		}
		if (err)
		{
			LOG("C139: RX failed - %d %s\n", err.value(), err.message());
		}
	}

	void start_connect()
	{
		if (m_stopping)
			return;

		std::error_code err;
		if (m_sock_tx.is_open())
			m_sock_tx.close(err);
		m_sock_tx.open(m_remoteaddr->protocol(), err);
		if (!err)
		{
			m_sock_tx.set_option(asio::ip::tcp::no_delay(true));
			m_sock_tx.set_option(asio::socket_base::keep_alive(true));
			osd_printf_verbose("C139: TX connecting to %s\n", *m_remoteaddr);
			m_timeout_tx.expires_after(std::chrono::seconds(10));
			m_timeout_tx.async_wait(
				[this](std::error_code const& err)
				{
					if (!err && m_state_tx.load() == 1)
					{
						osd_printf_verbose("C139: TX connect timed out\n");
						std::error_code e;
						m_sock_tx.close(e);
						m_state_tx.store(0);
						start_connect();
					}
				});
			m_sock_tx.async_connect(
				*m_remoteaddr,
				[this](std::error_code const& err)
				{
					m_timeout_tx.cancel();
					if (err)
					{
						osd_printf_verbose("C139: TX connect error - %d %s\n", err.value(), err.message());
						std::error_code e;
						m_sock_tx.close(e);
						m_state_tx.store(0);
						start_connect();
					}
					else
					{
						LOG("C139: TX connection established\n");
						m_state_tx.store(2);
					}
				});
			m_state_tx.store(1);
		}
	}

	void start_send_tx()
	{
		if (m_stopping)
			return;

		unsigned used = m_fifo_tx.read(&m_buffer_tx[0], std::min<unsigned>(m_fifo_tx.used(), m_buffer_tx.size()), true);
		m_sock_tx.async_write_some(
			asio::buffer(&m_buffer_tx[0], used),
			[this](std::error_code const& err, std::size_t length)
			{
				m_fifo_tx.consume(length);
				if (err)
				{
					LOG("C139: TX connection error: %s\n", err.message().c_str());
					m_sock_tx.close();
					m_state_tx.store(0);
					m_fifo_tx.clear();
					start_connect();
				}
				else if (m_fifo_tx.used())
				{
					start_send_tx();
				}
			});
	}

	void start_receive_rx()
	{
		if (m_stopping)
			return;

		m_sock_rx.async_read_some(
			asio::buffer(m_buffer_rx),
			[this](std::error_code const& err, std::size_t length)
			{
				if (err || !length)
				{
					if (err)
						LOG("C139: RX connection error: %s\n", err.message());
					else
						LOG("C139: RX connection lost\n");
					m_sock_rx.close();
					m_state_rx.store(0);
					m_fifo_rx.clear();
					start_accept();
				}
				else
				{
					if (m_fifo_rx.write(&m_buffer_rx[0], length) == UINT_MAX)
					{
						LOG("C139: RX buffer overflow\n");
						m_sock_rx.close();
						m_state_rx.store(0);
						m_fifo_rx.clear();
						start_accept();
						return;
					}
					if (m_forward)
						send(&m_buffer_rx[0], length);

					start_receive_rx();
				}
			});
	}

	template <typename Format, typename... Params>
	void logerror(Format&& fmt, Params&&... args) const
	{
		util::stream_format(
			std::cerr,
			"[%s] %s",
			m_device.tag(),
			util::string_format(std::forward<Format>(fmt), std::forward<Params>(args)...));
	}

	namco_c139_device &m_device;
	std::thread m_thread;
	asio::io_context m_ioctx;
	asio::executor_work_guard<asio::io_context::executor_type> m_work_guard{m_ioctx.get_executor()};
	std::optional<asio::ip::tcp::endpoint> m_localaddr;
	std::optional<asio::ip::tcp::endpoint> m_remoteaddr;
	asio::ip::tcp::acceptor m_acceptor;
	asio::ip::tcp::socket m_sock_rx;
	asio::ip::tcp::socket m_sock_tx;
	asio::steady_timer m_timeout_tx;
	bool m_stopping;
	bool m_forward;
	std::atomic_uint m_state_rx;
	std::atomic_uint m_state_tx;
	fifo m_fifo_rx;
	fifo m_fifo_tx;
	std::array<uint8_t, 0x400> m_buffer_rx;
	std::array<uint8_t, 0x400> m_buffer_tx;
};


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************
#define REG_0_STATUS 0
#define REG_1_MODE 1
#define REG_2_CONTROL 2
#define REG_3_START 3
#define REG_4_RXSIZE 4
#define REG_5_TXSIZE 5
#define REG_6_RXOFFSET 6
#define REG_7_TXOFFSET 7

// device type definition
DEFINE_DEVICE_TYPE(NAMCO_C139, namco_c139_device, "namco_c139", "Namco C139 Serial")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

void namco_c139_device::data_map(address_map &map)
{
	map(0x0000, 0x3fff).rw(FUNC(namco_c139_device::ram_r),FUNC(namco_c139_device::ram_w));
}

void namco_c139_device::regs_map(address_map &map)
{
	map(0x00, 0x0f).rw(FUNC(namco_c139_device::reg_r), FUNC(namco_c139_device::reg_w));
}


//-------------------------------------------------
//  namco_c139_device - constructor
//-------------------------------------------------

namco_c139_device::namco_c139_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NAMCO_C139, tag, owner, clock),
	m_irq_cb(*this)
{
	auto const &opts = mconfig.options();

	m_localhost = opts.comm_localhost();
	m_localport = opts.comm_localport();
	m_remotehost = opts.comm_remotehost();
	m_remoteport = opts.comm_remoteport();
	m_forward = false;

	// come up with some magic number for identification
	std::string remotehost = util::string_format("%s:%s", m_remotehost, m_remoteport);
	m_linkid = 0;
	for (int x = 0; x < sizeof(remotehost) && remotehost[x] != 0; x++)
	{
		m_linkid ^= remotehost[x];
	}

	LOG("C139: ID byte = %02d\n", m_linkid);

	std::fill(std::begin(m_buffer), std::end(m_buffer), 0);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void namco_c139_device::device_start()
{
	m_timer_12mhz = timer_alloc(FUNC(namco_c139_device::timer_12mhz_callback), this);
	m_timer_12mhz->adjust(attotime::never);

	auto ctx = std::make_unique<context>(*this);
	m_context = std::move(ctx);
	m_context->start();

	// state saving
	save_item(NAME(m_ram));
	save_item(NAME(m_reg));

	save_item(NAME(m_linkid));

	save_item(NAME(m_irq_state));
	save_item(NAME(m_irq_count));

	save_item(NAME(m_txblock));
	save_item(NAME(m_txdelay));
	save_item(NAME(m_rxdelay));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void namco_c139_device::device_reset()
{
	std::fill(std::begin(m_ram), std::end(m_ram), 0);
	std::fill(std::begin(m_reg), std::end(m_reg), 0);

	m_context->reset(m_localhost, m_localport, m_remotehost, m_remoteport, m_forward);

	m_timer_12mhz->adjust(attotime::from_hz(12_MHz_XTAL), 0, attotime::from_hz(12_MHz_XTAL));

	m_reg[REG_0_STATUS] = 0x0000;
	m_reg[REG_1_MODE] = 0x000f;
	m_reg[REG_2_CONTROL] = 0x0000;
	m_reg[REG_3_START] = 0x0000;
	m_reg[REG_4_RXSIZE] = 0x0000;
	m_reg[REG_5_TXSIZE] = 0x0000;
	m_reg[REG_6_RXOFFSET] = 0x1000;
	m_reg[REG_7_TXOFFSET] = 0x0000;

	m_irq_state = CLEAR_LINE;
	m_irq_count = 0x0000;

	m_txblock = 0x0000;
	m_txdelay = 0x0000;
	m_rxdelay = 0x0000;
}

void namco_c139_device::device_stop()
{
	m_timer_12mhz->adjust(attotime::never);

	m_context->stop();
	m_context.reset();

	m_irq_state = CLEAR_LINE;

	m_txblock = 0x0000;
	m_txdelay = 0x0000;
	m_rxdelay = 0x0000;
}


//**************************************************************************
//  READ/WRITE HANDLERS
//**************************************************************************

uint16_t namco_c139_device::ram_r(offs_t offset)
{
	return m_ram[offset];
}

void namco_c139_device::ram_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_ram[offset]);
	m_ram[offset] &= 0x01ff;
}

uint16_t namco_c139_device::reg_r(offs_t offset)
{
	uint16_t result = m_reg[offset];
	switch (offset)
	{
		case REG_0_STATUS:
			if (m_reg[REG_5_TXSIZE] == 0)
				result |= 0x4;
			if (m_reg[REG_4_RXSIZE] == 0)
				result |= 0x8;
			break;

		case REG_6_RXOFFSET:
			// rx offset cannot go below 0x1000
			result = m_reg[offset] | 0x1000;
			break;

		default:
			break;
	}
	if (!machine().side_effects_disabled())
		LOG("C139: reg_r[%02x] = %04x\n", offset, result);
	return result;
}

void namco_c139_device::reg_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (!machine().side_effects_disabled())
		LOG("C139: reg_w[%02x] = %04x\n", offset, data);

	// registers are mirrored and limited in size
	offset &= 0x07;
	switch (offset)
	{
		case REG_0_STATUS:
		case REG_1_MODE:
			data &= 0x000f;
			break;

		case REG_2_CONTROL:
		case REG_3_START:
			data &= 0x0003;
			break;

		case REG_4_RXSIZE:
		case REG_5_TXSIZE:
			data &= 0x00ff;
			break;

		case REG_6_RXOFFSET:
		case REG_7_TXOFFSET:
			data &= 0x1fff;
			break;

		default:
			break;
	}
	m_reg[offset] = data;

	switch (offset)
	{
		case REG_0_STATUS:
			// status reset / irq ack?
			m_reg[REG_0_STATUS] = 0x00;
			m_irq_count = 0;
			m_irq_state = CLEAR_LINE;
			m_irq_cb(m_irq_state);
			break;

		case REG_5_TXSIZE:
			m_txblock = data * 12;
			break;

		default:
			break;
	}
}

void namco_c139_device::sci_de_hack(uint8_t data)
{
	// prepare "filenames"
	switch (data)
	{
		case 0:
			m_localhost = "127.0.0.1";
			m_localport = "15112";
			m_remotehost = "127.0.0.1";
			m_remoteport = "15113";
			break;
		case 1:
			m_localhost = "127.0.0.1";
			m_localport = "15113";
			m_remotehost = "127.0.0.1";
			m_remoteport = "15114";
			break;
		case 2:
			m_localhost = "127.0.0.1";
			m_localport = "15114";
			m_remotehost = "127.0.0.1";
			m_remoteport = "15112";
			m_forward = true;
			break;
		default:
			m_localhost = "127.0.0.1";
			m_localport = "15112";
			m_remotehost = "127.0.0.1";
			m_remoteport = "15112";
			break;
	}

	// come up with some magic number for identification
	std::string remotehost = util::string_format("%s:%s", m_remotehost, m_remoteport);
	m_linkid = 0;
	for (int x = 0; x < sizeof(remotehost) && remotehost[x] != 0; x++)
	{
		m_linkid ^= remotehost[x];
	}

	LOG("C139: ID byte = %02d\n", m_linkid);
}

// 12mhz clock input
TIMER_CALLBACK_MEMBER(namco_c139_device::timer_12mhz_callback)
{
	comm_tick();
}

void namco_c139_device::comm_tick()
{
	// hold int for a moment
	int m_new_state = m_irq_state;
	if (m_irq_count > 0)
		if (--m_irq_count == 0)
			m_new_state = CLEAR_LINE;

	switch (m_reg[REG_1_MODE])
	{
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
			if (m_reg[REG_4_RXSIZE] == 0x00 || m_reg[REG_5_TXSIZE] == 0x00)
			{
				// fire int if RXSIZE or TXSIZE is 0
				m_new_state = ASSERT_LINE;
				m_reg[REG_1_MODE] = 0x0f;
			}
			break;


		case 0x04:
		case 0x05:
			if (m_reg[REG_4_RXSIZE] == 0x00 || m_reg[REG_0_STATUS] & 0x02)
			{
				// fire int if RXSIZE = 0 OR sync-bit detected.
				m_new_state = ASSERT_LINE;
				m_reg[REG_1_MODE] = 0x0f;
			}
			break;


		case 0x06:
		case 0x07:
			if (m_reg[REG_4_RXSIZE] == 0x00)
			{
				// fire int if RXSIZE = 0.
				m_new_state = ASSERT_LINE;
				m_reg[REG_1_MODE] = 0x0f;
			}
			break;


		case 0x08:
		case 0x09:
		case 0x0a:
		case 0x0b:
			if (m_reg[REG_5_TXSIZE] == 0x00)
			{
				// fire int if TXSIZE = 0.
				m_new_state = ASSERT_LINE;
				m_reg[REG_1_MODE] = 0x0f;
			}
			break;


		case 0x0c:
		case 0x0d:
			if (m_reg[REG_0_STATUS] & 0x02)
			{
				// fire int if sync-bit detected.
				m_new_state = ASSERT_LINE;
				m_reg[REG_1_MODE] = 0x0f;
			}
			break;


		case 0x0e:
		case 0x0f:
			// do nothing?
			break;

		default:
			break;
	}

	if (m_irq_state != m_new_state)
	{
		m_irq_count = 4;
		m_irq_state = m_new_state;
		m_irq_cb(m_irq_state);
	}

	if (m_txblock > 0)
		m_txblock--;

	// prevent completing send too fast
	if (m_txdelay > 0)
		if (--m_txdelay == 0)
			m_reg[REG_5_TXSIZE] = 0;

	// prevent receiving too fast
	if (m_rxdelay > 0)
		m_rxdelay--;

	unsigned data_size = 0x200;
	if (m_txblock == 0 && m_txdelay == 0)
		send_data(data_size);

	if (m_rxdelay == 0)
		read_data(data_size);
}

void namco_c139_device::read_data(unsigned data_size)
{
	// try to read a message
	unsigned recv = read_frame(data_size);
	if (recv > 0)
	{
		// save message to "rx buffer"
		unsigned rx_size = m_buffer[0x1ff];
		unsigned rx_offset = m_reg[REG_6_RXOFFSET]; // rx offset in words
		LOG("C139: rx_offset = %04x, rx_size == %02x\n", rx_offset, rx_size);
		unsigned buf_offset = 0;
		for (unsigned j = 0x00; j < rx_size; j++)
		{
			uint16_t data = get_u16be(&m_buffer[buf_offset]);
			m_ram[0x1000 + (rx_offset & 0x0fff)] = data;

			// check sync-bit
			if (data & 0x0100)
				m_reg[REG_0_STATUS] |= 0x02;

			rx_offset++;
			buf_offset += 2;
		}

		// update regs
		m_reg[REG_4_RXSIZE] -= rx_size;
		m_reg[REG_6_RXOFFSET] += rx_size;

		// prevent overflow
		m_reg[REG_4_RXSIZE] &= 0x00ff;
		m_reg[REG_6_RXOFFSET] &= 0x0fff;

		m_rxdelay = rx_size * 12;
	}
}

unsigned namco_c139_device::read_frame(unsigned data_size)
{
	unsigned bytes_read = m_context->receive(&m_buffer[0], data_size);
	if (bytes_read == UINT_MAX)
	{
		// ignore errors
		return 0;
	}
	return bytes_read;
}

void namco_c139_device::send_data(unsigned data_size)
{
	// check if tx is halted
	if (m_reg[REG_3_START] & 0x01)
		return;

	if (m_reg[REG_5_TXSIZE] == 0x00)
		return;

	unsigned tx_offset = m_reg[REG_7_TXOFFSET]; // tx offset in words
	unsigned tx_mask = 0x1fff;
	unsigned tx_size = m_reg[REG_5_TXSIZE];
	LOG("C139: tx_mode = %02x, tx_offset = %04x, tx_size == %02x\n", m_reg[REG_1_MODE], tx_offset, tx_size);

	m_buffer[0x1fe] = m_linkid;
	m_buffer[0x1ff] = tx_size;

	// mode 8 (ridgera2) has sync bit set in data (faulty)
	// mode 8 (raverace) has sync bit set in data (faulty)
	// mode c (ridgeracf) has no sync bit set in data (faulty)
	// mode 9 (acedrive) has sync bit set in data (correctly)
	bool use_sync_bit = m_reg[REG_1_MODE] & 0x01;

	unsigned buf_offset = 0;
	for (unsigned j = 0x00; j < tx_size; j++)
	{
		uint16_t data = m_ram[tx_offset & tx_mask];
		if (!use_sync_bit)
			data &= 0x00ff;
		put_u16be(&m_buffer[buf_offset], data);
		tx_offset++;
		buf_offset += 2;
	}

	// set bit-8 on last byte (mode 8/c)
	if (!use_sync_bit)
		m_buffer[buf_offset -2] |= 0x01;

	//m_reg[REG_5_TXSIZE] = 0x00;
	m_txdelay = tx_size * 12;

	send_frame(data_size);
}

void namco_c139_device::send_frame(unsigned data_size)
{
	unsigned bytes_sent = m_context->send(&m_buffer[0], data_size);
	if (bytes_sent == UINT_MAX)
	{
		// ignore errors
	}
}
