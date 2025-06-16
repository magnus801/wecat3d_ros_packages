# wecat3d_ros_packages

this creates a pcd_file named "merged_encoder_output.pcd" and opens it to write into the pcd

    std::ofstream pcd_file("merged_encoder_output.pcd", std::ios::out | std::ios::trunc);
    if (!pcd_file.is_open()) {
        std::cerr << "Failed to open PCD file for writing!" << std::endl;
        return 1;
    }
    
sets a placeholder value for pcd in height and points as we dont know how many points we need to set for pcd writing
     A counter is set for total_points which counts total points after each scan,the total number of points after each scan is 1280

    std::streampos header_start = pcd_file.tellp();
    pcd_file << "# .PCD v0.7 - Point Cloud Data file\n";
    pcd_file << "VERSION 0.7\n";
    pcd_file << "FIELDS x y z\n";
    pcd_file << "SIZE 4 4 4\n";
    pcd_file << "TYPE F F F\n";
    pcd_file << "COUNT 1 1 1\n";
    pcd_file << "WIDTH 0000000000\n";  // Placeholder for point count (10 digits)
    pcd_file << "HEIGHT 1\n";
    pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    pcd_file << "POINTS 0000000000\n";  // Placeholder for point count (10 digits)
    pcd_file << "DATA ascii\n";
    std::streampos data_start = pcd_file.tellp();
    
    uint64_t total_points = 0;


Loads library and sets encoder configs laser scan is taken after each 20 ticks from encoder

    if (!loadEthernetScannerLibrary()) {
            std::cerr << "Failed to load EthernetScanner library!" << std::endl;
            pcd_file.close();
            return 1;
        }
        
        try {
            Sensor sensor("192.168.100.1", 32001);  
            
            sensor.connect(0); 
            sensor.write_data("SetHeartbeat=1000");
            sensor.write_data("SetExposureTime=750");
            sensor.write_data("SetTriggerSource=2");
            sensor.write_data("SetTriggerEncoderStep=20");
            sensor.write_data("SetEncoderTriggerFunction=2");
            sensor.write_data("ResetEncoder");
            sensor.write_data("SetRangeImageNrProfiles=1");
            sensor.write_data("SetAcquisitionStart");
        


        while (!stop) {
            try {
Sets profile buffer and timeout and counter for scan function, scanno increases by 1 after everytime getscan function is running

                sensor.get_scanned_profile(scan,1000); 
                scanNo++;
                
This sets x,y,z values of the scan function

                const std::vector<float>& x_values = scan.roiWidthX;
                const std::vector<float>& z_values = scan.roiHeightZ;
                uint32_t encoder_unsigned = scan.encoderValue;

Following logic to convert 32bit unsigned int of encoder to 64bit signed int for negative values

                int64_t encoder = static_cast<int64_t>(encoder_unsigned);
                if (encoder_unsigned >= static_cast<uint32_t>(1ULL << 31)) {
                    encoder -= static_cast<int64_t>(1ULL << 32);
                }
Simple debugging print for printing delta value of encoder

                if (last_encoder != std::numeric_limits<int64_t>::min()) {
                    int64_t delta_encoder = encoder - last_encoder;
                    double delta_microns = delta_encoder * (ENC_SCALE_MM * 1000.0);
                }
Main logic for taking profile if{} block checks if encoder tick has changed

                if (last_encoder == std::numeric_limits<int64_t>::min() || encoder != last_encoder) {
                    double y_value = encoder * ENC_SCALE_MM;
                    
After every each laser scan which contains 1280 points the for{} block runs 1280 times to write from array x_values,z_values and current encoder value

                    for (size_t i = 0; i < x_values.size(); ++i) {
                        // Write in ASCII format: x y z
                        pcd_file << std::fixed << std::setprecision(6) 
                                << x_values[i] << " " 
                                << y_value << " " 
                                << z_values[i] << "\n";
                    }

This 1280 points are written into the pcd file and total_points is updated by 1280

                    pcd_file.flush(); // Ensure data is written immediately
                    total_points += x_values.size();

Sets the current ROS2 timestamp on the message also checks if point count changed from previous scan and resizes message buffer if needed

                    msg.header.stamp = node->now();
                    if (msg.width != x_values.size()) {
                        msg.width = x_values.size();
                        mod.resize(x_values.size());
                    }

                    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
                    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
                    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z"); 

Converts millimeters to meters    

                    float y_scaled = y_value * scale_factor;

This loop fills the Ros2 Pointcloud2 message with 3D point data and converts mmto meters for ros2

                    for (size_t i = 0; i < x_values.size(); ++i, ++it_x, ++it_y, ++it_z) {
                        *it_x = x_values[i] * scale_factor;      
                        *it_y = y_scaled;                        
                        *it_z = z_values[i] * scale_factor;     
                    }
                    
                    pub->publish(msg);
                    
print statment to keep check of no of points
                    if (scanNo % 500 == 0) { 
                        std::cout << "Processed " << scanNo << " scans, total points: " << total_points << std::endl;
                    }
                }

Remembers current encoder position for next iteration

                last_encoder = encoder;

If the encoder is not moving the sensor throws exception for -1 this logic handles the exception

            } catch (const SensorException& e) {
                // Continue on timeout errors (-1) which was for no new profile recieved
                if (std::string(e.what()).find("-1") != std::string::npos) {
                    continue;
                } else {
                    throw;
                }
            }
        }


For example if total points recorded 483840 this 483840/1280= 378 scans means this many times encoder position changes,So  378*20=7,560 actual encoder steps of movement with .