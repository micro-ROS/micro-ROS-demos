# How check Orion NGSIv2 updated value

**Linux**

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

> **Note:** only curl application is required.
