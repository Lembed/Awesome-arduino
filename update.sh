#!/bin/bash

INFILE='README.md'
OUTFILE='.tmp'

usage()
{
    echo ""
    echo " usage:"
    echo ""
    echo "./update.sh <cmd>"
    echo "    where <cmd> is one of:"
    echo "      --install-or-update     (does full installation or update.)"
    echo "      --remove 		        (removes all installed)"
    echo ""
    echo "example:"
    echo '    $ ./update.sh --install-or-update'
}

function check
{

	git --version >/dev/null
	if [ $? -ne 0 ] 
	then
		echo "no git command found ! please download and install"
		exit
	fi
	:
}

function pure_context
{
	# zero list
	>$2

	for LINE in  $(cat $1)
	do
		rs=$(echo $LINE | grep "github" )
		if [[ "$rs" != "" ]]; then
			rt=$(echo $LINE | grep "Projects" )
			if [[ "$rt" !=  "" ]]; then
				continue
			fi
			rl=$(echo $LINE | awk -F '(' '{print $2}' | awk -F ')' '{print $1}' )
			echo "$rl" >> $OUTFILE
		fi
			:
	done
}



function is_git 
{
	isg=$(cat $1/.git/config)
	if [[ "$isg" ==  "" ]]; then
		return "0"
	else
		return "1"
	fi
}


function do_update
{	

	for LINE in  $(cat $1)
	do
		instant=$(echo $LINE | awk -F '/' '{print $5}' )
		echo $instant
		if [[ "$instant" ==  "" ]]; then
			continue
			echo "noting"
		fi

		if [ -d "$instant" ]; then
			ig=is_git "$instant"
			if [[ "$ig" ==  "0" ]]; then
				rm -rf $instant
				git clone "$LINE" &
			fi
			git pull origin master ; echo "$instant"
		else
			git clone "$LINE" &
		fi
		:
	done
}

function do_del
{	

	for LINE in  $(cat $1)
	do
		instant=$(echo $LINE | awk -F '/' '{print $5}' )
		echo $instant
		if [[ "$instant" ==  "" ]]; then
			continue
		fi
		echo $instant
		if [ -d "$instant" ]; then
			rm -rf $instant	
		fi
		:
	done
}


# main
check

if [ $# -eq 1 -a "$1" == "--install-or-update" ]; then
    pure_context $INFILE $OUTFILE
	do_update $OUTFILE
	rm -rf $OUTFILE
    exit
fi

if [ $# -eq 1 -a "$1" == "--remove" ]; then
    pure_context $INFILE $OUTFILE
	do_del $OUTFILE
	rm -rf $OUTFILE
    exit
fi


usage