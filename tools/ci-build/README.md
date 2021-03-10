# Building fog_sw packages
```
cd <your-fog_sw-parent-dir>
docker build -t fogsw_artifacts -f ./Dockerfile ../../..
container_id=$(docker create fogsw_artifacts "")
docker cp ${container_id}:/packages .
docker rm ${container_id}

```

