// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#include "externalinput.h"

#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <QtEndian>
#include <QDebug>

#include <cstring>

/*
 * Wire format (little endian, 96 bytes):
 *   0  char[4]  magic "DPLY"
 *   4  u8       version (1)
 *   5  u8       flags   (bit0: force neutral)
 *   6  u16      reserved
 *   8  u32      sequence (monotonic, wrap-safe compare)
 *  12  u8[16]   key (shared secret from CHIAKI_EXTINPUT_KEY)
 *  28  u32      buttons (ChiakiControllerButton bit layout)
 *  32  u8       l2, 33 u8 r2, 34 u16 reserved
 *  36  i16      left_x, left_y, right_x, right_y
 *  44  2x { i8 id, u8 pad, u16 x, u16 y }
 *  56  f32      gyro xyz, accel xyz, orient xyzw
 */

static float ReadF32(const uchar *p)
{
	quint32 v = qFromLittleEndian<quint32>(p);
	float f;
	std::memcpy(&f, &v, sizeof(f));
	return f;
}

ExternalInput *ExternalInput::CreateFromEnvironment(QObject *parent)
{
	bool ok = false;
	int port = qEnvironmentVariableIntValue("CHIAKI_EXTINPUT_PORT", &ok);
	QByteArray key = QByteArray::fromHex(qgetenv("CHIAKI_EXTINPUT_KEY"));
	if(!ok || port <= 0 || port > 65535)
		return nullptr;
	if(key.size() != 16)
	{
		qWarning() << "ExternalInput: CHIAKI_EXTINPUT_KEY must be 32 hex chars, external input disabled";
		return nullptr;
	}
	auto *ext = new ExternalInput((quint16)port, key, parent);
	if(!ext->enabled)
	{
		delete ext;
		return nullptr;
	}
	return ext;
}

ExternalInput::ExternalInput(quint16 port, const QByteArray &key, QObject *parent)
	: QObject(parent), key(key)
{
	chiaki_controller_state_set_idle(&state);
	socket = new QUdpSocket(this);
	// Loopback only: never reachable from the network.
	if(!socket->bind(QHostAddress::LocalHost, port))
	{
		qWarning() << "ExternalInput: failed to bind 127.0.0.1:" << port << socket->errorString();
		return;
	}
	connect(socket, &QUdpSocket::readyRead, this, &ExternalInput::ReadPending);
	watchdog = new QTimer(this);
	watchdog->setTimerType(Qt::PreciseTimer);
	connect(watchdog, &QTimer::timeout, this, &ExternalInput::WatchdogTick);
	watchdog->start(WATCHDOG_MS / 4);
	enabled = true;
	qInfo() << "ExternalInput: listening on 127.0.0.1:" << port << "- local controller sources are ignored";
}

void ExternalInput::ReadPending()
{
	bool changed = false;
	while(socket->hasPendingDatagrams())
	{
		QByteArray data;
		data.resize((int)socket->pendingDatagramSize());
		QHostAddress sender;
		socket->readDatagram(data.data(), data.size(), &sender);
		if(!sender.isLoopback())
			continue;
		if(ParsePacket(data))
			changed = true;
	}
	if(changed)
		emit StateChanged();
}

bool ExternalInput::ParsePacket(const QByteArray &data)
{
	if(data.size() != PACKET_SIZE)
		return false;
	const uchar *p = reinterpret_cast<const uchar *>(data.constData());
	if(std::memcmp(p, "DPLY", 4) != 0 || p[4] != 1)
		return false;

	// constant-time key compare
	uchar diff = 0;
	for(int i = 0; i < 16; i++)
		diff |= p[12 + i] ^ (uchar)key[i];
	if(diff)
		return false;

	quint32 seq = qFromLittleEndian<quint32>(p + 8);
	if(have_seq && (qint32)(seq - last_seq) <= 0)
		return false; // stale or replayed
	have_seq = true;
	last_seq = seq;
	last_packet.start();
	timed_out = false;

	ChiakiControllerState s;
	chiaki_controller_state_set_idle(&s);
	if(!(p[5] & 1))
	{
		s.buttons = qFromLittleEndian<quint32>(p + 28) & 0xffff;
		s.l2_state = p[32];
		s.r2_state = p[33];
		s.left_x = qFromLittleEndian<qint16>(p + 36);
		s.left_y = qFromLittleEndian<qint16>(p + 38);
		s.right_x = qFromLittleEndian<qint16>(p + 40);
		s.right_y = qFromLittleEndian<qint16>(p + 42);
		for(int i = 0; i < CHIAKI_CONTROLLER_TOUCHES_MAX; i++)
		{
			const uchar *t = p + 44 + i * 6;
			s.touches[i].id = (int8_t)t[0];
			s.touches[i].x = qFromLittleEndian<quint16>(t + 2);
			s.touches[i].y = qFromLittleEndian<quint16>(t + 4);
		}
		s.gyro_x = ReadF32(p + 56); s.gyro_y = ReadF32(p + 60); s.gyro_z = ReadF32(p + 64);
		s.accel_x = ReadF32(p + 68); s.accel_y = ReadF32(p + 72); s.accel_z = ReadF32(p + 76);
		s.orient_x = ReadF32(p + 80); s.orient_y = ReadF32(p + 84);
		s.orient_z = ReadF32(p + 88); s.orient_w = ReadF32(p + 92);
	}
	state = s;
	return true;
}

void ExternalInput::WatchdogTick()
{
	// Dead-man switch: if the mixer stops talking (crash, hang, closed),
	// release everything instead of holding the last state forever.
	if(!timed_out && last_packet.isValid() && last_packet.elapsed() > WATCHDOG_MS)
	{
		timed_out = true;
		chiaki_controller_state_set_idle(&state);
		qWarning() << "ExternalInput: no packets for" << WATCHDOG_MS << "ms, forcing neutral state";
		emit StateChanged();
	}
}
