# Execute demo

This document procedures you a quick method of how execute this demo.

## Run orion broker

To run a context broker in the host system execute the below subshell script.

```shell
(
mkdir orion
cd orion
echo "mongo:
    image: mongo:3.4
    command: --nojournal
orion:
    image: fiware/orion
    links:
        - mongo
    ports:
        - \"1026:1026\"
    command: -dbhost mongo" > docker-compose.yml
sudo docker-compose up
)
```

> **Note:** The only requirement is to have installed [docker CE](https://docs.docker.com/install/) y [docker compose](https://docs.docker.com/compose/install/).

> **Note:** Context broker docker is only available on Linux.

> **Note:** Context broker will be listening on localhost:1026

> **Note:** All below steps have been extracted from the docker hub [official FIWARE repository](https://hub.docker.com/r/fiware/orion).


## Run FIROS2 node

To run the demo exectute the below script.

```bash
. WORK_SPACE_PATH/install/./local_setup.bash
WORK_SPACE_PATH/install/firos2/bin/firos2 WORK_SPACE_PATH/install/int32_firos2/lib/config.xml
```

> **Note:** WORK_SPACE_PATH is the path of the ROS2 workspace.


## Check Orion NGSIv2 updated value

### Using a Linux terminal

Execute this subshell script in a linux terminal to create the entity and consult the value

```shell
(
    URL_PORT="localhost:1026"

    ENTITY="Helloworld"
    ATTRIBUTE="count"
    TYPE="Helloworld"
    ATTRIBUTE_TYPE="Number"
    ATTRIBUTE_VALUE="0"

    UPDATE_TIME="0.5"

    curl  -v \
        --include \
        --header 'Content-Type: application/json' \
        --request POST \
        --data-binary '{  "id": "$ENTITY", 
                            "type": "$TYPE", 
                            "$ATTRIBUTE": { 
                                "value": "$ATTRIBUTE_VALUE", 
                                "type": "$ATTRIBUTE_TYPE" 
                            }
                        }' \
        '$URL_PORT/v2/entities'



    (
        while (( 1 ))
        do 
        curl -v "$URL_PORT/v2/entities/$ENTITY/attrs/$ATTRIBUTE/value?type=$TYPE"
        echo ""
        sleep $UPDATE_TIME
        done
    )
)
```

> **Note:** The script is prepared to be copied into the clipboard and paste on a Linux terminal.

> **Note:** curl application is required.


### Using a web browser

Visit the url direction in your favorite web browser.

```html
localhost:1026/v2/entities/Helloworld/attrs/count/value?type=Helloworld"
```


