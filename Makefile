all: build

build:
	rosdep install -i --from-path src --rosdistro iron -y
	colcon build

build-container:
	make -C container build

deploy: build-container
	docker run --rm -it --name ecosun \
		--privileged \
		-v $(CURDIR):/ecosun:z \
		-v /dev:/dev \
		quay.io/mmoltras/ecosun:latest

teardown:
	docker rm -f ecosun

.PHONY: deploy teardown build-container build
