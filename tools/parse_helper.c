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

static const char precedence[] = {
	/* (  +  -  *  /  &  ^  |  ) */
	   0, 2, 2, 1, 1, 3, 4, 5, 6
};
static const char unary_operations[]  = "(+-";
static const char binary_operations[] = " +-*/&^|)";

static uint32_t do_func(uint32_t val1, uint32_t val2, int op)
{
	switch (op) {
	case 1:
		return val1 + val2;
	case 2:
		return val1 - val2;
	case 3:
		return val1 * val2;
	case 4:
		return val1 / val2;
	case 5:
		return val1 & val2;
	case 6:
		return val1 ^ val2;
	case 7:
		return val1 | val2;
	}
	fprintf(stderr, "Error: in func %s: val1=%d val2=%d op = %d\n",
			__func__, val1, val2, op);
	exit(EXIT_FAILURE);
}

static int find_op(char c, const char *p)
{
	int i;
	for (i = 0; ; i++) {
		if (c == p[i])
			return i;
		if (!p[i])
			break;
	}
	return -1;
}

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

int ph_get_value(struct parse_helper *ph, uint32_t *pval)
{
	char *endptr;
	int op_i = 0;
	int val_i = 0;
	unsigned char op[16];
	uint32_t val[16];
	int unary = 1;
	char *p;

	p = ph->p;
	for (;;) {
		char c;
		int i, j;
		const char *ops = unary ? unary_operations : binary_operations;

		if (unary) {
			ph->p = p;
			if (ph_skip_separators(ph))
				return -1;
			p = ph->p;
			c = *p;
		} else {
			for (;;) {
				c = *p;
				if ((c != ' ') && (c != '\t'))
					break;
				p++;
			}
		}
		i = find_op(c, ops);
		debug("%d,%c,%d:%s\n", i, c, unary, p);
		if ((i < 0) && unary) {
			if (val_i >= ARRAY_SIZE(val))
				return -1;
			errno = 0;
			val[val_i++] = strtoul(p, &endptr, 16);
			if (errno || (p == endptr)) {
				ph->p = p;
				return -1;
			}
			p = endptr;
			unary = 0;
			debug("val[%d]=%x,%d,%d\n", val_i - 1, val[val_i - 1],
					op_i, val_i);
do_unary:
			while (op_i) {
				j = op[op_i - 1];
				if (!(j & 0x80))
					break;
				op_i--;
				val[val_i - 1] = do_func(0,
						val[val_i - 1], j & 0x7f);
				debug("un:%d,%x,%d,%d\n", val[val_i - 1], j,
						op_i, val_i);
			}
			continue;
		}
		if (i < 0) {
			c = 0;
			i = 8;
		} else {
			p++;
		}
		if (c == '(') {
			if (op_i >= ARRAY_SIZE(op))
				return -1;
			op[op_i++] = i;
			debug("op[%d]=%x,%d,%d\n", op_i - 1, op[op_i - 1],
					op_i, val_i);
			unary = 1;
			continue;
		}
		for (;;) {
			if (!op_i || unary)
				break;
			j = op[op_i - 1];
			if (j == 0) {
				if (c == ')') {
					op_i--;
					goto do_unary;
				}
				break;
			}
			if ((j & 0x80)) {
				op_i--;
				val[val_i - 1] = do_func(0,
						val[val_i - 1], j & 0x7f);
				debug("unary:%d,%x\n", val[val_i - 1], j);
				continue;
			}
			if (precedence[i] < precedence[j])
				break;
			if (val_i < 2)
				return -1;
			op_i--;
			val[val_i - 2] = do_func(val[val_i - 2],
					val[val_i - 1], j);
			val_i--;
			debug("binary:%d,%x,%d,%d\n", val[val_i - 1], j,
					op_i, val_i);
		}
		if (c == ')') {
			fprintf(stderr, "Error: unmatched parenthesis\n");
			return -1;
		}
		if (i == 8) {
			if ((op_i != 0) || (val_i != 1)) {
				fprintf(stderr, "Error: syntax %d %d\n",
						op_i, val_i);
				return -1;
			}
			ph->p = p;
			*pval = val[0];
			return 0;
		}
		if (op_i >= ARRAY_SIZE(op))
			return -1;
		op[op_i++] = i | (unary << 7);
		debug("op[%d]=%x,%d,%d\n", op_i - 1, op[op_i - 1], op_i, val_i);
		unary = 1;
	}
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

