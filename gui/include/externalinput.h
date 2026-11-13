// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL
// DuoPlay external controller input for chiaki-ng.
//
// When the environment variables CHIAKI_EXTINPUT_PORT and CHIAKI_EXTINPUT_KEY
// (32 hex chars) are set, chiaki-ng listens on 127.0.0.1:<port> for externally
// mixed controller states and forwards ONLY those to the console, ignoring its
// own controller/keyboard sources. Without these variables nothing changes.

#ifndef CHIAKI_EXTERNALINPUT_H
#define CHIAKI_EXTERNALINPUT_H

#include <chiaki/controller.h>

#include <QObject>
#include <QByteArray>
#include <QElapsedTimer>

class QUdpSocket;
class QTimer;

class ExternalInput : public QObject
{
	Q_OBJECT

	public:
		static constexpr int PACKET_SIZE = 96;
		static constexpr int WATCHDOG_MS = 200;

		// Returns nullptr when external input is not configured.
		static ExternalInput *CreateFromEnvironment(QObject *parent);

		bool IsEnabled() const { return enabled; }
		ChiakiControllerState GetState() const { return state; }

	signals:
		void StateChanged();

	private:
		ExternalInput(quint16 port, const QByteArray &key, QObject *parent);
		void ReadPending();
		void WatchdogTick();
		bool ParsePacket(const QByteArray &data);

		QUdpSocket *socket = nullptr;
		QTimer *watchdog = nullptr;
		QElapsedTimer last_packet;
		QByteArray key;
		quint32 last_seq = 0;
		bool have_seq = false;
		bool enabled = false;
		bool timed_out = true;
		ChiakiControllerState state;
};

#endif // CHIAKI_EXTERNALINPUT_H
