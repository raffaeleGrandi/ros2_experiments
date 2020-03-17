class Constants:

    class Comms:
        
        MAX_MESSAGE_SIZE = 1000
        SENDER_INPUT_TOPIC_TAG = 'input'
        SENDER_OUTPUT_TOPIC_TAG = 'output'
        SYS_TOPIC = 'system_topic'
        STOP_MESSAGE = '!STOP!'

    class System:
        DESTROY_DELAY = 1
        SYSTEM_DEFAULT_QOS = 0