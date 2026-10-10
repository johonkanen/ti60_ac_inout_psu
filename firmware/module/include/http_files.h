/*
 * Files of a directory on a mounted filesystem, managed over HTTP:
 *   GET    <prefix>/        list the files as JSON
 *   POST   <prefix>/<name>  write the request body to <name>, replacing it
 *   DELETE <prefix>/<name>  remove <name>
 * The directory is CONFIG_HTTP_FILES_ROOT. Names are single path components.
 * Serve the files with a static_fs resource on the same directory, e.g. the service's
 * fallback, so a web client can be replaced without rebuilding the image.
 *
 *   HTTP_RESOURCE_DEFINE(files, my_service, "/files/" "*", &http_files_resource);
 */
#ifndef HTTP_FILES_H
#define HTTP_FILES_H

#include <zephyr/net/http/server.h>

extern struct http_resource_detail_dynamic http_files_resource;

#endif
