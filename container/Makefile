all: build

build:
	docker build \
		--tag quay.io/mmoltras/ecosun \
		-f Containerfile \
		$(CURDIR)/..

.PHONY: build
