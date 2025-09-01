/*
COMMENTS (MATTHEW):
 - rename pht1 and gps1 TOP BAY
 - should median be used for rocket, or should we 
 - at start of all the if-else statements, do:

    if( abs(PHT1.alt) - abs(PHT2.alt) >= ERROR_RANGE){ // BASICALLY IF THE PHT START DIVERGING TOO FAR FROM EACHOTHER
        PRIMARY_ALTIMETER;
    }
        elif( GPS.alt - ERROR_RANGE <= PHT1.ALT <= GPS.alt + ERROR_RANGE){
            PRIMARY_ALTIMETER = ALT
        }
            elif( GPS.alt - ERROR_RANGE <= PHT1.ALT <= GPS.alt + ERROR_RANGE){
            PRIMARY_ALTIMETER = ALT
        }
*/

case LaunchState::Ignition_to_Apogee:
    {
        static int count = 0;
        
        // Read sensor values
        double PHT1_alt = alt1.getAltitude();
        double PHT2_alt = alt2.getAltitude() + HEIGHT_OFFSET;
        double GPS1_alt = gps1.getAltitude();
        double GPS2_alt = gps2.getAltitude() + HEIGHT_OFFSET;

        // Error detection
        bool PHT1_error = (PHT1_alt == 0);
        bool PHT2_error = (PHT2_alt == 0);
        bool GPS1_error = gps1.readingCheck();
        bool GPS2_error = gps2.readingCheck();

        // Valid altitudes (non-error)
        std::vector<double> validReadings;
        if (!PHT1_error)
            validReadings.push_back(PHT1_alt);
        if (!PHT2_error)
            validReadings.push_back(PHT2_alt);
        if (!GPS1_error)
            validReadings.push_back(GPS1_alt);
        if (!GPS2_error)
            validReadings.push_back(GPS2_alt);

        double final_altitude = 0;

        if (validReadings.size() >= 3)
        {
            // Find the median of the valid readings
            std::sort(validReadings.begin(), validReadings.end());
            double median_alt = (validReadings.size() % 2 == 0) ? (validReadings[validReadings.size() / 2 - 1] + validReadings[validReadings.size() / 2]) / 2.0
                                                                : validReadings[validReadings.size() / 2];

            // Check if three sensors agree within ERROR_RANGE
            int agreement_count = 0;
            for (double alt : validReadings)
            {
                if (fabs(alt - median_alt) < ERROR_RANGE)
                {
                    agreement_count++;
                }
            }

            if (agreement_count >= 3)
            {
                final_altitude = median_alt; // Ignore the outlier
            }
            else
            {
                // If two agree and two don't, favor the top bay sensors (PHT1, GPS1)
                if ((fabs(PHT1_alt - GPS1_alt) < ERROR_RANGE) && (!PHT1_error && !GPS1_error))
                {
                    final_altitude = (PHT1_alt + GPS1_alt) / 2.0;
                }
                else if ((fabs(PHT2_alt - GPS2_alt) < ERROR_RANGE) && (!PHT2_error && !GPS2_error))
                {
                    final_altitude = (PHT2_alt + GPS2_alt) / 2.0;
                }
                else
                {
                    // Otherwise, prioritize PHT sensors
                    if (!PHT1_error && !PHT2_error)
                    {
                        final_altitude = (PHT1_alt + PHT2_alt) / 2.0;
                    }
                    else if (!PHT1_error)
                    {
                        final_altitude = PHT1_alt;
                    }
                    else if (!PHT2_error)
                    {
                        final_altitude = PHT2_alt;
                    }
                    else
                    {
                        // Last fallback to GPS
                        if (!GPS1_error && !GPS2_error)
                        {
                            final_altitude = (GPS1_alt + GPS2_alt) / 2.0;
                        }
                        else if (!GPS1_error)
                        {
                            final_altitude = GPS1_alt;
                        }
                        else if (!GPS2_error)
                        {
                            final_altitude = GPS2_alt;
                        }
                    }
                }
            }
        }
        else
        {
            // Not enough valid readings, fallback logic
            if (!PHT1_error && !PHT2_error)
            {
                final_altitude = (PHT1_alt + PHT2_alt) / 2.0;
            }
            else if (!PHT1_error)
            {
                final_altitude = PHT1_alt;
            }
            else if (!PHT2_error)
            {
                final_altitude = PHT2_alt;
            }
            else if (!GPS1_error && !GPS2_error)
            {
                final_altitude = (GPS1_alt + GPS2_alt) / 2.0;
            }
            else if (!GPS1_error)
            {
                final_altitude = GPS1_alt;
            }
            else if (!GPS2_error)
            {
                final_altitude = GPS2_alt;
            }
        }

        // Store the altitude values for rate of change calculation
        AltArray[count % READINGS_LENGTH] = final_altitude;
        count++;

        // Step 5: Check for apogee based on calculated rate of change
        if (count >= READINGS_LENGTH)
        {
            double rate_of_change_1 = calculateRateOfChange(AltArray, READINGS_LENGTH);
            // double rate_of_change_2 = calculateRateOfChange(AltArray2, READINGS_LENGTH);

            if ((fabs(rate_of_change_1) < RATE_THRESHOLD && rate_of_change_1 < 0))
            {
                Serial.println("Halya has reached apogee!");
                current_state = LaunchState::Thousand_ft;
            }
            count = 0;
        }

        delay(100); // Delay for sensor updates
        break;
    }