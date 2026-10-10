/*
 * Raw TCP transmit rate: a server on port 5003 that sends a 64 KB buffer over and over
 * to whoever connects, until the client closes. The client counts (tcp_tx_client.py).
 * The same network settings as the scope app, without its capture, copies or pacing.
 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#define PORT  5003
#define CHUNK (64 * 1024)

static uint8_t buffer[CHUNK];

int main(void)
{
	struct sockaddr_in address = {
		.sin_family = AF_INET,
		.sin_port = htons(PORT),
	};
	int server = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	for (int i = 0; i < CHUNK; i++) {
		buffer[i] = (uint8_t)i;
	}
	zsock_bind(server, (struct sockaddr *)&address, sizeof(address));
	zsock_listen(server, 1);
	printk("tcp_tx on %s: port %d\n", CONFIG_BOARD_TARGET, PORT);

	while (true) {
		int client = zsock_accept(server, NULL, NULL);
		uint64_t sent = 0;
		int64_t start = k_uptime_get();

		while (true) {
			ssize_t n = zsock_send(client, buffer, CHUNK, 0);

			if (n <= 0) {
				break;
			}
			sent += n;
		}
		int64_t ms = k_uptime_get() - start;

		printk("sent %llu bytes in %lld ms, %llu kB/s\n", sent, ms,
		       ms ? sent / (uint64_t)ms : 0);
		zsock_close(client);
	}
	return 0;
}
