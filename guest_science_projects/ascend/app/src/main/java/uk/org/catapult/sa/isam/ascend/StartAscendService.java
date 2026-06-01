
/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

package uk.org.catapult.sa.isam.ascend;

import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

// GS Library
import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.android.gs.StartGuestScienceService;

// GS API
//import gov.nasa.arc.astrobee.Kinematics;
//import gov.nasa.arc.astrobee.PendingResult;
//import gov.nasa.arc.astrobee.Result;
//import gov.nasa.arc.astrobee.types.ActionType;
//import gov.nasa.arc.astrobee.types.Point;
//import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class StartAscendService extends StartGuestScienceService {

    // The API implementation
    // ApiCommandImplementation api = null;

    /**
     * This function is called when the GS manager sends a custom command to your apk.
     * Please handle your commands in this function.
     *
     * @param command
     */
    @Override
    public void onGuestScienceCustomCmd(String command) {
        sendReceivedCustomCommand("info");

        try {
            // Transform the String command into a JSON object so we can read it
            JSONObject jsonCommand = new JSONObject(command);

            // Get the name of the command we received (see commands.xml files in res folder)
            String commandStr = jsonCommand.getString("name");

            Log.i("onGuestScienceCustomCmd", "Ascend received command " + commandStr + ".");

            switch (commandStr) {

                case "noOp":

                    sendData(MessageType.JSON, "data", "noOp");
                    break;

                case "doSomething":

                    Log.i("TEST", "Do something");

                    sendData(MessageType.JSON, "data", "doSomething");
                    break;
            }

        } catch (JSONException e) {
            // Send an error message to the GSM and GDS
            sendData(MessageType.JSON, "data", "ERROR parsing JSON");
        } catch (Exception ex) {
            // Send an error message to the GSM and GDS
            sendData(MessageType.JSON, "data", "Unrecognized ERROR");
        }
    }

    /**
     * This function is called when the GS manager starts your apk. Put all of your start up code in here.
     */
    @Override
    public void onGuestScienceStart() {

        // Get a unique instance of the Astrobee API in order to command the robot
        // api = ApiCommandImplementation.getInstance();
        // Log.i("TEST", "Passed getInstance");

        // Inform the GS Manager and the GDS that the app has been started
        sendStarted("info");
    }

    /**
     * This function is called when the GS manager stops your apk.
     * Put all of your clean up code in here. You should also call the terminate helper function
     * at the very end of this function.
     */
    @Override
    public void onGuestScienceStop() {
        // Stop the API
        //api.shutdownFactory();

        // Inform the GS manager and the GDS that this app stopped
        sendStopped("info");

        // Destroy all connection with the GS Manager
        terminate();
    }

}
