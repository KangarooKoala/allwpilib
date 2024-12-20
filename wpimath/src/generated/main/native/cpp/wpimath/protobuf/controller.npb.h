// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9 */

#ifndef PB_WPI_PROTO_CONTROLLER_NPB_H_INCLUDED
#define PB_WPI_PROTO_CONTROLLER_NPB_H_INCLUDED
#include <pb.h>
#include <span>
#include <string_view>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _wpi_proto_ProtobufArmFeedforward {
    static const pb_msgdesc_t* msg_descriptor(void) noexcept;
    static std::string_view msg_name(void) noexcept;
    static pb_filedesc_t file_descriptor(void) noexcept;

    double ks;
    double kg;
    double kv;
    double ka;
    double dt;
} wpi_proto_ProtobufArmFeedforward;

typedef struct _wpi_proto_ProtobufDifferentialDriveFeedforward {
    static const pb_msgdesc_t* msg_descriptor(void) noexcept;
    static std::string_view msg_name(void) noexcept;
    static pb_filedesc_t file_descriptor(void) noexcept;

    double kv_linear;
    double ka_linear;
    double kv_angular;
    double ka_angular;
} wpi_proto_ProtobufDifferentialDriveFeedforward;

typedef struct _wpi_proto_ProtobufElevatorFeedforward {
    static const pb_msgdesc_t* msg_descriptor(void) noexcept;
    static std::string_view msg_name(void) noexcept;
    static pb_filedesc_t file_descriptor(void) noexcept;

    double ks;
    double kg;
    double kv;
    double ka;
    double dt;
} wpi_proto_ProtobufElevatorFeedforward;

typedef struct _wpi_proto_ProtobufSimpleMotorFeedforward {
    static const pb_msgdesc_t* msg_descriptor(void) noexcept;
    static std::string_view msg_name(void) noexcept;
    static pb_filedesc_t file_descriptor(void) noexcept;

    double ks;
    double kv;
    double ka;
    double dt;
} wpi_proto_ProtobufSimpleMotorFeedforward;

typedef struct _wpi_proto_ProtobufDifferentialDriveWheelVoltages {
    static const pb_msgdesc_t* msg_descriptor(void) noexcept;
    static std::string_view msg_name(void) noexcept;
    static pb_filedesc_t file_descriptor(void) noexcept;

    double left;
    double right;
} wpi_proto_ProtobufDifferentialDriveWheelVoltages;


/* Initializer values for message structs */
#define wpi_proto_ProtobufArmFeedforward_init_default {0, 0, 0, 0, 0}
#define wpi_proto_ProtobufDifferentialDriveFeedforward_init_default {0, 0, 0, 0}
#define wpi_proto_ProtobufElevatorFeedforward_init_default {0, 0, 0, 0, 0}
#define wpi_proto_ProtobufSimpleMotorFeedforward_init_default {0, 0, 0, 0}
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_init_default {0, 0}
#define wpi_proto_ProtobufArmFeedforward_init_zero {0, 0, 0, 0, 0}
#define wpi_proto_ProtobufDifferentialDriveFeedforward_init_zero {0, 0, 0, 0}
#define wpi_proto_ProtobufElevatorFeedforward_init_zero {0, 0, 0, 0, 0}
#define wpi_proto_ProtobufSimpleMotorFeedforward_init_zero {0, 0, 0, 0}
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_init_zero {0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define wpi_proto_ProtobufArmFeedforward_ks_tag  1
#define wpi_proto_ProtobufArmFeedforward_kg_tag  2
#define wpi_proto_ProtobufArmFeedforward_kv_tag  3
#define wpi_proto_ProtobufArmFeedforward_ka_tag  4
#define wpi_proto_ProtobufArmFeedforward_dt_tag  5
#define wpi_proto_ProtobufDifferentialDriveFeedforward_kv_linear_tag 1
#define wpi_proto_ProtobufDifferentialDriveFeedforward_ka_linear_tag 2
#define wpi_proto_ProtobufDifferentialDriveFeedforward_kv_angular_tag 3
#define wpi_proto_ProtobufDifferentialDriveFeedforward_ka_angular_tag 4
#define wpi_proto_ProtobufElevatorFeedforward_ks_tag 1
#define wpi_proto_ProtobufElevatorFeedforward_kg_tag 2
#define wpi_proto_ProtobufElevatorFeedforward_kv_tag 3
#define wpi_proto_ProtobufElevatorFeedforward_ka_tag 4
#define wpi_proto_ProtobufElevatorFeedforward_dt_tag 5
#define wpi_proto_ProtobufSimpleMotorFeedforward_ks_tag 1
#define wpi_proto_ProtobufSimpleMotorFeedforward_kv_tag 2
#define wpi_proto_ProtobufSimpleMotorFeedforward_ka_tag 3
#define wpi_proto_ProtobufSimpleMotorFeedforward_dt_tag 4
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_left_tag 1
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_right_tag 2

/* Struct field encoding specification for nanopb */
#define wpi_proto_ProtobufArmFeedforward_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   ks,                1) \
X(a, STATIC,   SINGULAR, DOUBLE,   kg,                2) \
X(a, STATIC,   SINGULAR, DOUBLE,   kv,                3) \
X(a, STATIC,   SINGULAR, DOUBLE,   ka,                4) \
X(a, STATIC,   SINGULAR, DOUBLE,   dt,                5)
#define wpi_proto_ProtobufArmFeedforward_CALLBACK NULL
#define wpi_proto_ProtobufArmFeedforward_DEFAULT NULL

#define wpi_proto_ProtobufDifferentialDriveFeedforward_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   kv_linear,         1) \
X(a, STATIC,   SINGULAR, DOUBLE,   ka_linear,         2) \
X(a, STATIC,   SINGULAR, DOUBLE,   kv_angular,        3) \
X(a, STATIC,   SINGULAR, DOUBLE,   ka_angular,        4)
#define wpi_proto_ProtobufDifferentialDriveFeedforward_CALLBACK NULL
#define wpi_proto_ProtobufDifferentialDriveFeedforward_DEFAULT NULL

#define wpi_proto_ProtobufElevatorFeedforward_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   ks,                1) \
X(a, STATIC,   SINGULAR, DOUBLE,   kg,                2) \
X(a, STATIC,   SINGULAR, DOUBLE,   kv,                3) \
X(a, STATIC,   SINGULAR, DOUBLE,   ka,                4) \
X(a, STATIC,   SINGULAR, DOUBLE,   dt,                5)
#define wpi_proto_ProtobufElevatorFeedforward_CALLBACK NULL
#define wpi_proto_ProtobufElevatorFeedforward_DEFAULT NULL

#define wpi_proto_ProtobufSimpleMotorFeedforward_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   ks,                1) \
X(a, STATIC,   SINGULAR, DOUBLE,   kv,                2) \
X(a, STATIC,   SINGULAR, DOUBLE,   ka,                3) \
X(a, STATIC,   SINGULAR, DOUBLE,   dt,                4)
#define wpi_proto_ProtobufSimpleMotorFeedforward_CALLBACK NULL
#define wpi_proto_ProtobufSimpleMotorFeedforward_DEFAULT NULL

#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   left,              1) \
X(a, STATIC,   SINGULAR, DOUBLE,   right,             2)
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_CALLBACK NULL
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_DEFAULT NULL

/* Maximum encoded size of messages (where known) */
#define WPI_PROTO_CONTROLLER_NPB_H_MAX_SIZE      wpi_proto_ProtobufArmFeedforward_size
#define wpi_proto_ProtobufArmFeedforward_size    45
#define wpi_proto_ProtobufDifferentialDriveFeedforward_size 36
#define wpi_proto_ProtobufDifferentialDriveWheelVoltages_size 18
#define wpi_proto_ProtobufElevatorFeedforward_size 45
#define wpi_proto_ProtobufSimpleMotorFeedforward_size 36


#endif
