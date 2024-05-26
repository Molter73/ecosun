all: deploy

deploy:
	docker run --rm -it --name ecosun \
		-e ROS_DOMAIN_ID=10 \
		-v $(CURDIR):/ecosun:z \
		--workdir /ecosun \
		osrf/ros:iron-desktop

teardown:
	docker rm -f ecosun

.PHONY: deploy teardown
