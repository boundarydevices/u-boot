/*
 * (C) Copyright 20012  Boundary Devices Inc, troy.kisky@boundarydevices.com
 *
 * Licensed under the GPL-2 or later.
 */

/* Required to obtain the getline prototype from stdio.h */
#define _GNU_SOURCE

#include "mkimage.h"
#include <image.h>
#include "parse_helper.h"

int ph_open(struct parse_helper *ph, char *filename)
{
	ph->line = NULL;
	ph->len = 0;
	ph->fd = fopen(filename, "r");
	ph->lineno = 0;
	ph->cmd_started = 0;
	ph->filename = filename;
	ph->p = NULL;
	return (!ph->fd) ? -1 : 0;
}

void ph_close(struct parse_helper *ph)
{
	fclose(ph->fd);
	ph->fd = NULL;
}

int ph_skip_separators(struct parse_helper *ph)
{
	int line_no = ph->lineno;
	char *p = ph->p;

	for (;;) {
		char c;
		if (!p) {
			if (getline(&ph->line, &ph->len, ph->fd) <= 0)
				return -1;
			ph->lineno++;
			p = ph->line;
			if (ph->cmd_started) {
				fprintf(stderr, "warning: continuing command on"
						" next line, line %s[%d](%s)\n",
						ph->filename, ph->lineno, p);
			}
		}
		c = *p;
		if ((c == ' ') || (c == '\t')) {
			p++;
			continue;
		}
		/* Drop all text starting with '#' as comments */
		if ((c == '#') || (c == '\r') || (c == '\n')
				|| !c) {
			p = NULL;
			continue;
		}
		if (c == ';') {
			if (ph->cmd_started) {
				fprintf(stderr, "Error: command not "
						"finished:%s[%d](%s)\n",
						ph->filename, ph->lineno, p);
				exit(EXIT_FAILURE);
			}
			p++;
			continue;
		}
		if (!ph->cmd_started && line_no == ph->lineno) {
			fprintf(stderr, "Error: extra data at end "
					"of line %s[%d](%s)\n",
					ph->filename, ph->lineno, p);
			exit(EXIT_FAILURE);
		}
		ph->p = p;
		return 0;
	}
}

int ph_skip_comma(struct parse_helper *ph)
{
	char *p = ph->p;

	for (;;) {
		char c = *p++;
		if ((c == '#') || (c == '\r') || (c == '\n') || !c)
			return 0;
		if (c == ',') {
			ph->p = p;
			ph_skip_separators(ph);
			return 1;
		}
		if ((c != ' ') && (c == '\t'))
			return 0;
	}
}

int ph_get_value(struct parse_helper *ph, uint32_t *pval)
{
	char *endptr;
	uint32_t value;

	if (ph_skip_separators(ph))
		return -1;
	errno = 0;
	value = strtoul(ph->p, &endptr, 16);
	if (errno || (ph->p == endptr))
		return -1;
	*pval = value;
	ph->p = endptr;
	return 0;
}

/*
 * Comma separator optional
 * Input:
 * ph - input source
 * data - array to fill in
 * cnt - exact number of elements to parse
 * Return: number of elements parsed, or error
 */
int ph_get_array(struct parse_helper *ph, uint32_t *data, int cnt)
{
	int i = 0;

	for (;;) {
		int ret = ph_get_value(ph, &data[i++]);
		if (ret)
			return ret;
		if (i >= cnt)
			break;
		ph_skip_comma(ph);		/* comma is optional */
	}
	return i;
}

static char *grab_token(char *dest, int size, char *src)
{
	while (size) {
		char c = *src;
		if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n')
				|| (c == '#') || !c)
			break;
		*dest++ = c;
		size--;
		src++;
	}
	if (!size)
		return NULL;
	*dest = 0;
	return src;
}

int ph_get_table_entry_id(struct parse_helper *ph,
		const table_entry_t *table, const char *table_name)
{
	int val;
	char token[16];
	char *p;

	if (ph_skip_separators(ph))
		return -1;
	p = grab_token(token, sizeof(token), ph->p);
	if (!p)
		return -1;
	val = get_table_entry_id(table, table_name, token);
	if (val != -1)
		ph->p = p;
	return val;
}

