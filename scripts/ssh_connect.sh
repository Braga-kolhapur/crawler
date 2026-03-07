#!/bin/bash

USER="greatsheep"
HOST="192.168.1.10"
PASSWORD="esr@12321"

sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no $USER@$HOST