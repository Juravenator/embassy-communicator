SHELL:=/bin/bash
.DEFAULT_GOAL := test

.PHONY: proto
proto:
	@mkdir -p proto
	protoc $(shell find ../proto -type d | xargs -I\{} echo --proto_path=\{}) --python_out=. $(shell find ../proto -name '*.proto')

.PHONY: test
test: proto
	python3 -m pip install -r requirements.txt
	python3 main.py