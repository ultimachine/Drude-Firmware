#!/bin/bash
make 2>&1 | grep --color=always -E "error:|$"
