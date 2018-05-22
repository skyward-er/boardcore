/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef TMTC_MESSAGE_HANDLERS_H
#define TMTC_MESSAGE_HANDLERS_H

#include <map>
#include "TMTC_Config.h"

namespace HomeoneBoard
{
namespace TMTC
{

/* Define the pair <MESSAGEID, COMMANDID> to identify a command */
typedef struct msgEntry_type
{
    uint8_t messageId;  // Mavlink message type id
    uint8_t commandId;  // 0 if there's no command in the message

    /* The < operator is used to define an order */
    bool operator<(msgEntry_type const& other) const
    {
        if (messageId < other.messageId)
        {
            return true;
        }
        else if (messageId == other.messageId)
        {
            if (commandId < other.commandId)
            {
                return true;
            }
        }
        return false;
    }
} msgEntry_t;

/* Define the pair <EVENTID, TOPICID> */
typedef struct evtEntry_type
{
    Events event;
    Topics topicID;
} evtEntry_t;

/**
 * This class wraps up the components needed to handle incoming commands.
 *
 * Every member of the class is static, so the class itself has no need to be
 * instantiated (it can be accessed using directly
 * MessageHandler::function()).
 */
class MessageHandler
{

private:
    /**
     * Map that contains an <EventID, TopicID> pair for each command that can
     * be directly mapped to an event.
     *
     * The key of the map is defined by a <MessageID, CommandID> pair, which
     * represents a single command.
     * Note that the commandId is specific for each type of Mavlink messages
     * and is set to 0 when the message has no internal command id.
     * This is how an entry looks like:
     * {
     *   {<type ID of the Mavlink message>, <command ID contained (or 0)>},
     *   {<ID of event to be posted>      , <Topic where to post it>}
     * }
     */
    static std::map<msgEntry_t, evtEntry_t> commandTranslationMap;

    /**
     * Searches a msgEntry in the map: if the map doesn't contain it, returns
     * false.
     * \param key               command to be searched in the map.
     * \param retrievedEntry    where to return the map's value.
     * \return                  true if the map contains the key.
     */
    static bool retrieveEvtEntry(msgEntry_t key, evtEntry_t* retrievedEntry);

public:
    /**
     * Handles the given Mavlink message according to its type id.
     * \param msg    pointer to the Mavlink message that was received.
     */
    static void handleMavlinkMessage(const mavlink_message_t* msg);
};
}
}

#endif /* TMTC_MESSAGE_HANDLERS_H */
