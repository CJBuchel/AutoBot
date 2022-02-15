#!/bin/bash

(cd ./Robot; ./gradlew $1 $2 $3)
(cd ./Sim; ./gradlew $1 $2 $3)