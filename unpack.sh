#!/bin/bash

ls $1/**/*.tar* | rg "(.*)\.tar.*" -r '$1' | xargs -I % sh -c 'mkdir % || true'
ls $1/**/*.tar* | rg "(.*)\.tar.*" -r '$1' | xargs -I % sh -c 'tar -xvf %.tar.gz -C %'
