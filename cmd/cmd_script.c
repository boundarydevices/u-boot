
// ## hyphop ## for khadas

/*

## script

simple plain script run from mem without mkimage wrappers

`script` is cool alternative for `autoscript` and `source` , we can
 use one `script` command for plain script and wrapped scripts!

+ https://github.com/hyphop/uboot-extra
+ https://raw.githubusercontent.com/hyphop/uboot-extra/master/cmd_script.c

## sintax and parsing

```
#!script - fist script line skip if no_chk == 0
72bytes  - mkimage script header same skip
##END##  - its end marker - after this lines all strings ignored
'\0'     - its same end marker

```

script parsed by run_command_list

## how to install it 

just add next line to Makefile

    obj-y += cmd_script.o

## uboot usage

    script [addr|check] [bytes] [no_chk] [silent] - run script starting at addr
        bytes - read bytes (hex) limit
        no_chk - no check script header dont igrnore 1st line
        silent - be silent

    script check && echo ok # check script cmd

## uboot usage  examples

    script 0x1000000				- simple run from addr 0x1000000 s
    script 0x1000000 32 			- same but only fist 32 bytes
    script 0x1000000 $filesize 			- same but limited by file size value 
    script 0x1000000 $filesize 1 		- same but no header
    script 0x1000000 $filesize 0 1		- silent

    # tftp script usage
    ADDR=0x1000000; tftp $ADDR test.script && script $ADDR 

    # usage as files
    ADDR=100000; ext4load mmc 1:5 $ADDR dhcp.cmd_test.script; script $ADDR $filesize

    # spi flash usage
    ADDR=100000; sf read $ADDR $SCRIPT_OFFSET $SCRIPT_BYTES; script $ADDR

*/


#include <common.h>
#include <command.h>
//#include <image.h>
#include <malloc.h>
//mainline
#include <mapmem.h>
//#include <asm/byteorder.h>

#define MAX_SCRIPT_SIZE 32768

int
script (ulong addr , ulong leng,  ulong no_hdr_chk , ulong silent)
{
	ulong  len = leng;
	void *buf;
	
	buf = map_sysmem(addr, 0);
	
	char *data;
	char *n;
	
	data = (char *)buf;
	n = data;

// simple mkimage header parser
	if ( *(n+0) == 0x27 &&
	     *(n+1) == 0x05 &&
	     *(n+2) == 0x19 &&
	     *(n+3) == 0x56
	     ) {

// sizes calculate
	    unsigned int l  = (*(n+14))*256 + *(n+15);
	    unsigned int l2 = (*(n+66))*256 + *(n+67);

// check headers sizes only // its not crc check
	    if ( l != l2 + 8 ) {
		printf ("** Script wrong headers size %u != %u **\n", l, l2  ) ;
		return -1;
	    }
// check zero script
	    if ( l2 == 0 ) {
		printf ("** Script zero size **\n" );
		return -1;
	    }

// fix offest
	    data+=72;

// fix len
	    len=l2;

// ignore after marker
	char *p = strstr( data , "\n##END##" );
	if (p != NULL) {
	    len = p - data;
	}

// info
	    if (!silent)
		printf ("## mkimage Script run a:%08lx l:%lu\n", addr, len) ;

// run
	    return run_command_list(data, len, 0);

// mkimage script end

	}
	
// simple script header parser skip first line if not comment
	if ( !no_hdr_chk ){
	if ( *n != '#') {
	    for (int i=0;1;i++) {
		if ( i == 128 || *(n+i) == '\0') {
		    printf ("** Script wrong header **\n") ;
		    return -1;
		}
		if ( *(n+i) == '\n') {
		    data+=i;
		    len-=i;
		    break;
		}
	    }
	}
	}

// OK

// truncate long script

	if (len > MAX_SCRIPT_SIZE ) {
	    len = MAX_SCRIPT_SIZE;
	    *(data + MAX_SCRIPT_SIZE) = '\0';
	}

// ignore after marker
	char *p = strstr( data , "\n##END##" );
	if (p != NULL) {
	    len = p - data;
	}

	if (!silent)
	    printf ("## Script run a:%08lx l:%lu c:%lu\n", addr, len, no_hdr_chk ) ;

	return run_command_list(data, len, 0);
	return 0;

}

/**************************************************/

int do_script (struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    int rcode;
    ulong addr, leng, hdr, slnt;
    char *fs;
    // just check
    if ( argc > 1 && *argv[1] == 'c' && *argv[1] == 'h' ) return 0;
    
    addr = simple_strtoul( argc < 2 || *argv[1] == '-' ?
	env_get("loadaddr") : argv[1], NULL, 16);
    
    leng = MAX_SCRIPT_SIZE;
    leng = 0;
    fs = env_get("filesize");
    if ( fs ) leng = simple_strtoul( fs , NULL, 16);
    leng = argc < 3 ? leng : *argv[2] == '-' ? leng : simple_strtoul(argv[2], NULL, 16);
    hdr  = argc < 4 ? 0 : simple_strtoul( argv[3], NULL, 10);
    slnt = argc < 5 ? 0 : simple_strtoul( argv[4], NULL, 10);
    
//	printf ("## Script cmd a:%08lx l:%lu c:%lu l:%lu fs:%s\n", addr, leng, hdr, slnt, fs);
    rcode = script (addr , leng , hdr, slnt);
    if ( slnt ) return 0;
    return rcode;
}

static char script_help_text[] =
	"[addr|check] [bytes] [no_chk] [silent] - run script starting at addr\n"
	"	bytes - read bytes (hex) limit\n"
	"	no_chk - no check header not ignore 1st line\n"
	"	silent - be silent\n";

U_BOOT_CMD(
	script, 5, 0,	do_script,
	"# run plain script from memory", script_help_text
);

U_BOOT_CMD(
	autoscr, 5, 0,	do_script,
	"# run plain script from memory", script_help_text
);
