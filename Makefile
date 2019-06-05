# DO NOT MODIFY - it is auto written from duckietown-env-developer

arch=arm32v7

branch=$(shell git rev-parse --abbrev-ref HEAD)

# name of the repo
repo=$(shell basename -s .git `git config --get remote.origin.url`)

tag=duckietown/$(arch)-$(repo):$(branch)

labels=$(shell ./labels.py)

build:
	docker build $(labels) -t $(tag) --build-arg ARCH=$(arch) --build-arg BRANCH=$(branch) .

push:
	docker push $(tag)

build-no-cache:
	docker build $(labels) --no-cache -t $(tag) .
