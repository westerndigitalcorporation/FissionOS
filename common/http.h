/*
 * http.h
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


#ifndef __HTTP_H__
#define __HTTP_H__

struct http_cgi_table;
typedef void (*http_cgi_cb_t)(struct netconn *client, struct netbuf *rxbuf, char **querystr, void *arg);
typedef struct http_cgi_table
{
    const char *name;
    http_cgi_cb_t callback;
    void *arg;
} http_cgi_table_t;

typedef struct http_page_table
{
    const char *name;
    const uint8_t *object;
    const uint32_t length;
} http_page_table_t;

extern const http_cgi_table_t http_cgi[];
extern const uint32_t http_cgi_table_count;

extern const http_page_table_t http_pages[];
extern const uint32_t http_page_table_count;

struct netconn *http_service_start(void);
void http_mainloop(struct netconn *http_netconn);

int http_request_key_value_next(char **querystr, char **key, char **value);
void http_content_type_send(struct netconn *client, const char *type);
void http_printf(struct netconn *client, const char *fmt, ...);

#endif /* __HTTP_H__ */
