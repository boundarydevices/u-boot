#!/usr/bin/awk -f
{
	s="00000000"$1;
	l=length(s);
	if(!((NR-1)%4))
		printf "%03x ",(NR-1)*4;
	for(i=l-1;i>l-8;i-=2)
		printf " %s",substr(s,i,2);
	if(!(NR%4))
		printf "\n";
}

