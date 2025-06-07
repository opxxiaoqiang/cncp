#!/usr/bin/env bash

# 设置 Mapbox 公钥和密钥
echo "pk.eyJ1IjoibWF3ZWl5dXdlaXdlaSIsImEiOiJjbGtrdjhhMHUwbWwzM3VwYzVtbXkwY2lrIn0.-u-QP8-tfrhAIpvQiNMsOw" > /data/params/d/MapboxPublicKey
echo "sk.eyJ1IjoibWF3ZWl5dXdlaXdlaSIsImEiOiJjbG15NHN1dDMwdWc5MmxwaDdkZ3Z5dHNyIn0.pk06qTKkAZyBC1Z37v8i0A" > /data/params/d/MapboxSecretKey

# 设置适当的权限
chmod 644 /data/params/d/MapboxPublicKey
chmod 644 /data/params/d/MapboxSecretKey

exec ./launch_chffrplus.sh
