/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
/*!
 * \file logger.h
 *
 * \brief Abstraction fo insulate our code as much as possible from
 *        from vendor specific logging implementaions.
 *
 * The Google glog logging package closely aligns with our logging needs
 * of being about to have a robust logging facility for develop and debugging
 * and being able to easily disable that for production use or when
 * integrated into PostgreSQL as an extension.
 *
 * While glog has quite a bit of functionality, we are mostly limiting our
 * current usage to the following:
 *
 * \code
    #include <glog/logging.h>

    FLAGS_log_dir = "./logs/";
    google::InitGoogleLogging("TrashCollection");
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::ERROR;
    FLAGS_minloglevel = google::INFO;

    DLOG(<level>) << "message";  // note no end line
    DLOG_IF(<level>, <cond>) << "message";
    DLOG_EVERY_N(<level>, <n>) << "message at count: " << google::COUNTER;
    CHECK(<cond>) << "message if cond is false and abort";

 * \endcode
 *
 * Where <level> is:
 * \arg \c 0 - \c INFO
 * \arg \c 1 - \c WARNING
 * \arg \c 2 - \c ERROR
 * \arg \c 3 - \c FATAL
 *
 * When compiled with \c NDEBUG the DLOG* statements are compiled away and
 * \c FATAL are changed to \c ERROR so no aborts will occur.
 *
 * Google glog can be downloaded from: https://code.google.com/p/google-glog/
 *
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <glog/logging.h>

#endif
