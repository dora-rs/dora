#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dora import Node

node = Node()

input_id, value, metadata = node.next()

print(f"id: {input_id}, value: {value}, metadata: {metadata}")
