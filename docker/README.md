# General guidelines for the use of competition development image

## To build the image

```
# cd docker
# ./build.sh
```

## To run the first instance of the image

```
# ./run.sh
```

## To join an existing instance

```
# ./join.sh

```

## To test that everything works

Build and start the container with,

```
# cd docker
# ./build.sh
# ./run.sh
```

and then in the container,

`$ roslaunch retail_store_simulation tiago_simulation`
