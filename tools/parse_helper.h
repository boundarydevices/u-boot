/*
 * (C) Copyright 20012  Boundary Devices Inc, troy.kisky@boundarydevices.com
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _PARSE_HELPER_H_
#define _PARSE_HELPER_H_

struct parse_helper {
	char *line;
	size_t len;
	FILE *fd;
	int lineno;
	char cmd_started;
	char *filename;
	char *p;
};

int ph_open(struct parse_helper *ph, char *filename);
void ph_close(struct parse_helper *ph);
int ph_skip_separators(struct parse_helper *ph);
int ph_skip_comma(struct parse_helper *ph);
int ph_get_value(struct parse_helper *ph, uint32_t *pval);
int ph_get_array(struct parse_helper *ph, uint32_t *data, int cnt);
int ph_get_table_entry_id(struct parse_helper *ph,
		const table_entry_t *table, const char *table_name);
#endif
