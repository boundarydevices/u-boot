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


#ifndef __INI_PROXY_H__
#define __INI_PROXY_H__

#ifdef __cplusplus
extern "C" {
#endif

void IniParserInit(void);
void IniParserUninit(void);
int IniParseFile(const char* filename);
int IniParseMem(unsigned char* file_buf);
int IniSetSaveFileName(const char* filename);
void IniParserFree(void);
void IniPrintAll(void);
void IniListSection(void);
const char* IniGetString(const char* section, const char* key, const char* def_value);
int IniSetString(const char *section, const char *key, const char *value);
int IniSaveToFile(const char *filename);

#ifdef __cplusplus
}
#endif

#endif //__INI_PROXY_H__
