#!/bin/bash
docker-compose -p ursim_alice_bob -f $(rospack find dual_ur5_driver)/resources/docker-compose.yml up
