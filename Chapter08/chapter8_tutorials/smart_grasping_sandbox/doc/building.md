
## Building the docker container
First clone the repository. Then go to the root of the cloned repository (where the README, Dockerfile, etc... are) and run:

```
docker build -t smart-grasping-sandbox .
```

You can now start the container:

```
docker run -it --name sgs --entrypoint /bin/bash -p 8080:8080 -p 7681:7681 -p 8181:8181 smart-grasping-sandbox
```
