#!/bin/bash

# the script get the master brach of the github repo, clone it to local with git
# or delete the repo directly.
# 
# for get pure url, first get the url list and print to tmp file, and will be delete
# at the end.
# 
# require : the script need to run in git bash
# 			or system with git installed
# 			
# the script use awk, grep, cat to do work
# 
# usage : sh update.sh
# 
INFILE='README.md'
OUTFILE='.tmp'

DEBUG=''

# print usage
function usage
{
    echo ""
    echo " usage:"
    echo ""
    echo "sh update.sh <cmd>"
    echo "    where <cmd> is one of:"
    echo "      --install-or-update     (does full installation or update.)"
    echo "      --remove 		        (removes all installed)"
    echo ""
    echo "example:"
    echo '    $ sh update.sh --install-or-update'
}

# for debug
function printvar
{
	if [[ "$DEBUG" != "" ]]; then
		echo $1
	fi
	:
}

# find the git is installed
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

# find and write the list to file
function pure_list
{
	# clean list
	>$2

	# loop the line of the readme
	for LINE in  $(cat $1)
	do
		rs=$(echo $LINE | grep "github" )
		if [[ "$rs" != "" ]]; then
			# the projects list can not be installed
			rt=$(echo $LINE | grep "Projects" )
			if [[ "$rt" !=  "" ]]; then
				continue
			fi
			rl=$(echo $LINE | awk -F '(' '{print $2}' | awk -F ')' '{print $1}' )
			# write list
			echo "$rl" >> $2
		fi
			:
	done
}


# find the git dir
function is_git
{
	isg=$(cat $1/.git/config)
	if [[ "$isg" ==  "" ]]; then
		return "0"
	else
		return "1"
	fi
}

# git clone the repo , update if it is exist
function do_update
{	
	# loop the line of the readme
	for LINE in  $(cat $1)
	do
		instant=$(echo $LINE | awk -F '/' '{print $5}' )
		printvar $instant
		if [[ "$instant" ==  "" ]]; then
			continue
			printvar "noting"
		fi

		if [ -d "$instant" ]; then
			# find the dir is git dir or not
			ig=is_git "$instant"
			if [[ "$ig" ==  "0" ]]; then
				rm -rf $instant
				git clone "$LINE" &
			fi
			git pull origin master ; printvar "$instant"
		else
			git clone "$LINE" &
		fi
		:
	done
}

# delete the repo, through the list in the list file, so it need to get 
# the list first
function do_del
{	

	for LINE in  $(cat $1)
	do
		instant=$(echo $LINE | awk -F '/' '{print $5}' )
		printvar $instant
		if [[ "$instant" ==  "" ]]; then
			continue
		fi
		printvar $instant
		if [ -d "$instant" ]; then
			rm -rf $instant	
		fi
		:
	done
}


# main
check

if [ $# -eq 1 -a "$1" == "--install-or-update" ]; then
	if [ ! -f "$OUTFILE" ]; then
    pure_list $INFILE $OUTFILE
	fi
	do_update $OUTFILE
	rm -rf $OUTFILE
    exit
fi

if [ $# -eq 1 -a "$1" == "--remove" ]; then

	if [ ! -f "$OUTFILE" ]; then
    pure_list $INFILE $OUTFILE
	fi

	do_del $OUTFILE
	rm -rf $OUTFILE
    exit
fi


usage