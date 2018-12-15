#include <iostream>

#include <std_msgs/msg/int32.hpp>

#include <rmw/rmw.h>

#include <NGSIv2/idl/JsonNGSIv2PubSubTypes.h>

#include <rosidl_typesupport_cpp/message_type_support.hpp>

#if defined(_WIN32) && defined (BUILD_SHARED_LIBS)
#if defined (_MSC_VER)
#pragma warning(disable: 4251)
#endif
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define  USER_LIB_EXPORT __declspec(dllexport)
#else
#define  USER_LIB_EXPORT __declspec(dllimport)
#endif
#else
#define USER_LIB_EXPORT
#endif


using eprosima::fastrtps::rtps::SerializedPayload_t;

extern "C" void USER_LIB_EXPORT transform(SerializedPayload_t *serialized_input, SerializedPayload_t *serialized_output){
    // Get type support
    const rosidl_message_type_support_t * type_support = rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::Int32>();

    // Convert to ROS2 serialized message
    rmw_serialized_message_t serialized_message;
    serialized_message.buffer = (uint8_t*)serialized_input->data;
    serialized_message.buffer_length = serialized_input->length;
    serialized_message.buffer_capacity = serialized_input->max_size;
    serialized_message.allocator = rcutils_get_default_allocator();

    // Desserizlize
    std_msgs::msg::Int32 data;
    if (rmw_deserialize(&serialized_message, type_support, (void*)&data) != RMW_RET_OK){
        return;
    }

    // Data display
    std::cout << "Data input: " << data.data << std::endl;

    // Custom transformation
    JsonNGSIv2 string_data;
    std:: string json = "{\"count\": { \"value\": " + std::to_string(data.data) + "} }";
    string_data.entityId("Helloworld"); // Fixed for the example
    string_data.data(json);
    std::cout << "Data output: " << string_data.data() << std::endl;

    // Serialization
    JsonNGSIv2PubSubType string_pst;
    serialized_output->reserve(string_pst.m_typeSize);
    string_pst.serialize(&string_data, serialized_output);
}

