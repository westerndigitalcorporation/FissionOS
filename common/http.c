/*
 * http.c
 *
 *
 * Copyright (c) 2013-2017 Western Digital Corporation or its affiliates.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the copyright holder nor the names of its contributors may not
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jeremy Garff <jeremy.garff@sandisk.com>
 *
 */


#include <stdint.h>
#include <stddef.h>

#include <lwip/ip.h>
#include <lwip/ip_addr.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <lwip/dhcp.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "http.h"

#define HTTP_MAX_URL                             512
#define HTTP_MAX_PRINT_LEN                       256

typedef struct
{
    const char *extension;
    const char *typestr;
} http_content_types_t;


const char http_file_not_found[] =
    "<!DOCTYPE html PUBLIC \"-//IETF//DTD HTML 2.0//EN\">"
    "<html>"
        "<head>"
            "<title>404 File Not Found</title>"
        "</head>"
        "<body>"
            "<h1>404 File Not Found</h1>"
        "</body>"
    "</html>";

const http_content_types_t http_content_types[] =
{
    { "html", "text/html; charset=UTF-8" },
    { "css",  "text/css" },
    { "js",   "text/javascript" },
    { "png",  "image/png" },
    { "txt",  "text/plain" },
    { "xml",  "application/xml" },
};
#define HTTP_CONTENT_TYPE_HTML                   0

// To save memory, this function will modify the string given to peel off
// the field and return a modified string and pointer.
static char *http_next_field(char **buf, char delim)
{
    char *next = NULL;
    char *current = *buf;

    // Strip leading characters
    while (*current)
    {
        if (*current != delim)
        {
            break;
        }
        current++;
    }

    next = current;
    while (*current)
    {
        if (*current == delim)
        {
            *current = 0;
            current++;
            break;
        }
        current++;
    }

    *buf = current;

    return next;
}

int http_request_key_value_next(char **querystr, char **key, char **value)
{
    char *pair = http_next_field(querystr, '&');

    if (!pair[0])
    {
        return -1;
    }

    *key = http_next_field(&pair, '=');
    *value = pair;

    return 0;
}

void http_printf(struct netconn *client, const char *fmt, ...)
{
    char buf[HTTP_MAX_PRINT_LEN];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, HTTP_MAX_PRINT_LEN - 1, fmt, ap);
    va_end(ap);

    netconn_write(client, buf, n, NETCONN_COPY);
}

/*
 * Figure out the content type based on the filename extension.
 */
static const char *http_content_type(const char *filename)
{
    const char *content_type = http_content_types[0].typestr;  // Default
    const char *filetype = filename;
    int i;

    while (*filetype)
    {
        if (*filetype++ == '.')
        {
            break;
        }
    }

    for (i = 0; i < (sizeof(http_content_types) / sizeof(http_content_types[0])); i++)
    {
        if (!strcmp(http_content_types[i].extension, filetype))
        {
            content_type = http_content_types[i].typestr;
            break;
        }
    }

    return content_type;
}

#define HTTP_RESULT_NONE                         -1
#define HTTP_RESULT_FILE                          0
#define HTTP_RESULT_CGI                           1
int http_lookup(const char *name, int *index)
{
    int i;

    for (i = 0; i < http_page_table_count; i++)
    {
        if (!strcmp(name, http_pages[i].name))
        {
            *index = i;
            return HTTP_RESULT_FILE;
        }
    }

    for (i = 0; i < http_cgi_table_count; i++)
    {
        if (!strcmp(name, http_cgi[i].name))
        {
            *index = i;
            return HTTP_RESULT_CGI;
        }
    }

    return HTTP_RESULT_NONE;
}

void http_content_type_send(struct netconn *client, const char *type)
{
    const char *content_type_fmt = "Content-type: %s\r\n\r\n";

    http_printf(client, content_type_fmt, type);
}

int http_default(struct netconn *client, struct netbuf *rxbuf, char **querystr)
{
    char *name = http_next_field(querystr, '?');
    const char *content_type = http_content_type(name);
    const char *ok = "HTTP/1.0 200 OK\r\n";
    int result, index;

    /*
     * See if we have a matching static page or cgi to return.
     */
    result = http_lookup(name, &index);
    switch (result)
    {
        case HTTP_RESULT_CGI:
            http_printf(client, ok);
            // Its up to the CGI to handle the rest of the stream, including deleting
            // the rxbuf we're handing it.  Its also up to the handler to send the
            // proper content-type string.
            http_cgi[index].callback(client, rxbuf, querystr, http_cgi[index].arg);
            return 0;

        case HTTP_RESULT_FILE:
            http_printf(client, ok);
            http_content_type_send(client, content_type);
            netconn_write(client, http_pages[index].object,
                          http_pages[index].length, NETCONN_NOCOPY);
            break;

        default:
            http_printf(client, "HTTP/1.0 404 Not Found\r\n");
            http_content_type_send(client, http_content_types[HTTP_CONTENT_TYPE_HTML].typestr);
            netconn_write(client, http_file_not_found, sizeof(http_file_not_found),
                          NETCONN_NOCOPY);
            break;
    }

    netbuf_delete(rxbuf);
	return 0;
}

static int http_url(struct netconn *client, struct netbuf *rxbuf, char *url_line)
{
    char *tmp, *def = "index.html";

    tmp = http_next_field(&url_line, ' ');  // Requeste type (GET/POST)
    tmp = http_next_field(&url_line, ' ');  // URL Request

    if (tmp[0] == '/') // Strip leading /
    {
        tmp++;
    }

    if (!tmp[0])  // No URL, use index.html
    {
        tmp = def;
    }

    return http_default(client, rxbuf, &tmp);
}

#define HTTP_STATE_TYPE                          0
#define HTTP_STATE_URL                           1
static int http_stream(struct netconn *client)
{
    struct netbuf *rx_buf;
    char url_line[HTTP_MAX_URL];
    int offset = 0;
    err_t err;

    err = netconn_recv(client, &rx_buf);
    while (err == ERR_OK)
    {
        uint8_t *data;
        uint16_t len;
        int i;

        netbuf_data(rx_buf, (void **)&data, &len);

        for (i = 0; i < len; i++)
        {
            switch (data[i])
            {
                case '\r':  // Strip these
                    break;

                case '\n':  // Line terminator
                    url_line[offset] = 0;
                    return http_url(client, rx_buf, url_line);

                default:    // Append to line
                    url_line[offset] = data[i];
                    offset++;
            }
        }

        if (netbuf_next(rx_buf) != ERR_OK)
        {
            netbuf_delete(rx_buf);   // Frees the netbuf list
            err = netconn_recv(client, &rx_buf);
        }
    }

    return 0;
}

void http_mainloop(struct netconn *http_netconn)
{
    while (1)
    {
        struct netconn *client;
        err_t err;

        err = netconn_accept(http_netconn, &client);
        if (err != ERR_OK)
            continue;

        http_stream(client);

        netconn_close(client);
        netconn_delete(client);
    }
}

struct netconn *http_service_start(void)
{
    struct netconn *http_netconn;
    err_t err;

    http_netconn = netconn_new(NETCONN_TCP);
    if (!http_netconn)
    {
        return NULL;
    }

    err = netconn_bind(http_netconn, IP_ADDR_ANY, 80);
    if (err != ERR_OK)
    {
        netconn_delete(http_netconn);
        return NULL;
    }
    err = netconn_listen(http_netconn);
    if (err != ERR_OK)
    {
        netconn_delete(http_netconn);
        return NULL;
    }

    return http_netconn;
}


