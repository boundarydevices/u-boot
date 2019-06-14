/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/


#ifndef __INI_HANDLER_H__
#define __INI_HANDLER_H__

#define CC_MAX_INI_FILE_NAME_LEN    (512)
#define CC_MAX_INI_FILE_LINE_LEN    (5120) //256->5k for large lcd_ext large init_on table

#define CC_MAX_INI_LINE_NAME_LEN    (128)
#define CC_MAX_INI_FILE_SIZE        (0x80000) //512KB

typedef struct S_INI_LINE {
    struct S_INI_LINE *pNext;
    char Name[CC_MAX_INI_LINE_NAME_LEN];
    char Value[CC_MAX_INI_FILE_LINE_LEN];
} INI_LINE;

typedef struct S_INI_SECTION {
    INI_LINE* pLine;
    INI_LINE* pCurLine;
    struct S_INI_SECTION *pNext;
    char Name[CC_MAX_INI_LINE_NAME_LEN];
} INI_SECTION;

typedef struct S_INI_HANDLER_DATA {
    INI_SECTION* mpFirstSection;
    INI_SECTION* mpCurSection;
    char mpFileName[CC_MAX_INI_FILE_NAME_LEN];
    void *data;
} INI_HANDLER_DATA;

#ifdef __cplusplus
extern "C" {
#endif

int ini_file_parse(const char* filename, INI_HANDLER_DATA *pHandlerData);
int ini_mem_parse(unsigned char* file_buf, INI_HANDLER_DATA *pHandlerData);
int ini_set_save_file_name(const char* filename, INI_HANDLER_DATA *pHandlerData);
void ini_free_mem(INI_HANDLER_DATA *pHandlerData);
void ini_print_all(INI_HANDLER_DATA *pHandlerData);
void ini_list_section(INI_HANDLER_DATA *pHandlerData);
const char* ini_get_string(const char* section, const char* key, const char* def_value, INI_HANDLER_DATA *pHandlerData);
int ini_set_string(const char *section, const char *key, const char *value, INI_HANDLER_DATA *pHandlerData);
int ini_save_to_file(const char *filename, INI_HANDLER_DATA *pHandlerData);

#ifdef __cplusplus
}
#endif

//for memory malloc & free debug
#define CC_MEMORY_ALLOC_FREE_TRACE              (1)
#define CC_MEMORY_ALLOC_FREE_TRACE_PRINT_ALL    (0)

#endif //__INI_HANDLER_H__
