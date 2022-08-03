void parse_config_data(const std::string &str, const std::string delimiters)
{
    boost::char_separator<char> sep(delimiters.c_str());
    boost::tokenizer<boost::char_separator<char>> tok(str, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();
    
    init_id.vehicle = *(itr++);
    init_id.bag_time = *(itr++);
    for(;itr!=tok.end(); itr++){
        init_id.sensor = *itr;
    }
    ROS_WARN("config read done");
}
void readConfiguration()
{
    std::string config_read_string = "";
    std::ifstream config_file(config_filename_);
    if (config_file.is_open())
    {
        while (config_file)
        {
            std::string s;
            getline(config_file, s);
            config_read_string += s + "\n";
        }
        config_file.close();
    }
    else
    {
        std::cout << "file open failed" << std::endl;
    }
    std::string delimiters = " \n\t";
    parse_config_data(config_read_string, delimiters);
}