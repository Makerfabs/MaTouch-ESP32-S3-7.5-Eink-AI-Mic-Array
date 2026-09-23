#pragma once

typedef enum {
    APP_EVENT_IMAGE_PREVIOUS = -1,
    APP_EVENT_IMAGE_NEXT = 1,
} app_event_type_t;

typedef struct {
    app_event_type_t type;
} app_event_t;
