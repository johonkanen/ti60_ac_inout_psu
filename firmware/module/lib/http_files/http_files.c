/*
 * Files of CONFIG_HTTP_FILES_ROOT over HTTP, see http_files.h.
 *
 * An upload goes to a hidden temporary file that is renamed over the old one when the
 * body is complete, so a failed or interrupted upload leaves the old file in place.
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <http_files.h>

LOG_MODULE_REGISTER(http_files, LOG_LEVEL_INF);

#define ROOT     CONFIG_HTTP_FILES_ROOT
#define NAME_MAX 64

static struct {
	bool active;
	struct fs_file_t file;
	char name[NAME_MAX + 1];
	char temp[sizeof(ROOT) + NAME_MAX + 2]; /* ROOT/.name */
	char path[sizeof(ROOT) + NAME_MAX + 1]; /* ROOT/name */
	size_t size;
	const char *error;
	int ret;
} upload;

static char reply[2048];
static struct fs_dirent entry;

/* the file name of the URL, after its last '/' and before any '?'; 0 when empty */
static int name_of(const char *url, char *name)
{
	const char *start = strrchr(url, '/');
	size_t len;

	start = start ? start + 1 : url;
	len = strcspn(start, "?");
	if (len == 0) {
		return 0;
	}
	if (len > NAME_MAX || start[0] == '.') {
		return -EINVAL;
	}
	for (size_t i = 0; i < len; i++) {
		char c = start[i];

		if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
		      c == '.' || c == '_' || c == '-')) {
			return -EINVAL;
		}
	}
	memcpy(name, start, len);
	name[len] = '\0';
	return len;
}

static void respond(struct http_response_ctx *response, enum http_status status, int len)
{
	response->status = status;
	response->body = reply;
	response->body_len = MIN(len, sizeof(reply) - 1);
	response->final_chunk = true;
}

static void respond_error(struct http_response_ctx *response, enum http_status status,
			  const char *error, int ret)
{
	respond(response, status,
		snprintf(reply, sizeof(reply), "{\"ok\":false,\"message\":\"%s (%d)\"}\n", error,
			 ret));
}

static void list(struct http_response_ctx *response)
{
	struct fs_dir_t dir;
	int len, ret;
	bool first = true;

	fs_dir_t_init(&dir);
	ret = fs_opendir(&dir, ROOT);
	if (ret) {
		respond_error(response, HTTP_500_INTERNAL_SERVER_ERROR, "cannot open " ROOT, ret);
		return;
	}
	len = snprintf(reply, sizeof(reply), "{\"ok\":true,\"files\":[");
	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
		if (entry.type != FS_DIR_ENTRY_FILE || entry.name[0] == '.' ||
		    len > sizeof(reply) - NAME_MAX - 64) {
			continue;
		}
		len += snprintf(reply + len, sizeof(reply) - len, "%s{\"name\":\"%s\",\"size\":%zu}",
				first ? "" : ",", entry.name, entry.size);
		first = false;
	}
	fs_closedir(&dir);
	len += snprintf(reply + len, sizeof(reply) - len, "]}\n");
	respond(response, HTTP_200_OK, len);
}

static void remove_file(const char *url, struct http_response_ctx *response)
{
	char name[NAME_MAX + 1];
	char path[sizeof(ROOT) + NAME_MAX + 1];
	int ret = name_of(url, name);

	if (ret <= 0) {
		respond_error(response, HTTP_400_BAD_REQUEST, "bad file name", ret);
		return;
	}
	snprintf(path, sizeof(path), ROOT "/%s", name);
	ret = fs_unlink(path);
	if (ret) {
		respond_error(response, ret == -ENOENT ? HTTP_404_NOT_FOUND
						       : HTTP_500_INTERNAL_SERVER_ERROR,
			      "cannot remove the file", ret);
		return;
	}
	LOG_INF("removed %s", path);
	respond(response, HTTP_200_OK,
		snprintf(reply, sizeof(reply), "{\"ok\":true,\"name\":\"%s\"}\n", name));
}

static void upload_fail(const char *error, int ret)
{
	if (!upload.error) {
		upload.error = error;
		upload.ret = ret;
		LOG_ERR("%s: %s (%d)", upload.path, error, ret);
	}
}

static void upload_start(const char *url)
{
	int ret;

	memset(&upload, 0, sizeof(upload));
	upload.active = true;
	fs_file_t_init(&upload.file);
	ret = name_of(url, upload.name);
	if (ret <= 0) {
		upload_fail("bad file name", ret);
		return;
	}
	snprintf(upload.path, sizeof(upload.path), ROOT "/%s", upload.name);
	snprintf(upload.temp, sizeof(upload.temp), ROOT "/.%s", upload.name);
	ret = fs_open(&upload.file, upload.temp, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret) {
		upload_fail("cannot create the file", ret);
	}
}

static void upload_end(bool complete, struct http_response_ctx *response)
{
	int ret;

	if (upload.file.mp) {
		ret = fs_close(&upload.file);
		if (ret) {
			upload_fail("cannot close the file", ret);
		}
	}
	if (complete && !upload.error) {
		ret = fs_rename(upload.temp, upload.path);
		if (ret) {
			upload_fail("cannot rename the file", ret);
		}
	}
	if (upload.error || !complete) {
		(void)fs_unlink(upload.temp);
	}
	if (response) {
		if (upload.error) {
			respond_error(response,
				      upload.ret == -ENOSPC ? HTTP_507_INSUFFICIENT_STORAGE
							    : HTTP_400_BAD_REQUEST,
				      upload.error, upload.ret);
		} else {
			LOG_INF("wrote %s, %zu bytes", upload.path, upload.size);
			respond(response, HTTP_200_OK,
				snprintf(reply, sizeof(reply),
					 "{\"ok\":true,\"name\":\"%s\",\"size\":%zu}\n",
					 upload.name, upload.size));
		}
	}
	upload.active = false;
}

static int files_cb(struct http_client_ctx *client, enum http_transaction_status status,
		    const struct http_request_ctx *request_ctx,
		    struct http_response_ctx *response_ctx, void *user_data)
{
	const char *url = (const char *)client->url_buffer;
	ssize_t written;

	ARG_UNUSED(user_data);

	if (status == HTTP_SERVER_TRANSACTION_ABORTED) {
		if (upload.active) {
			LOG_WRN("upload of %s aborted after %zu bytes", upload.path, upload.size);
			upload_end(false, NULL);
		}
		return 0;
	}
	if (status == HTTP_SERVER_TRANSACTION_COMPLETE) {
		return 0;
	}

	switch (client->method) {
	case HTTP_GET:
		list(response_ctx);
		return 0;
	case HTTP_DELETE:
		remove_file(url, response_ctx);
		return 0;
	default:
		break;
	}

	if (!upload.active) {
		upload_start(url);
	}
	if (request_ctx->data_len && !upload.error) {
		written = fs_write(&upload.file, request_ctx->data, request_ctx->data_len);
		if (written != request_ctx->data_len) {
			upload_fail("cannot write the file", written < 0 ? written : -ENOSPC);
		} else {
			upload.size += written;
		}
	}
	if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
		upload_end(true, response_ctx);
	}
	return 0;
}

struct http_resource_detail_dynamic http_files_resource = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET) | BIT(HTTP_POST) |
						     BIT(HTTP_DELETE),
		.content_type = "application/json",
	},
	.cb = files_cb,
};
