#include "idmind_behaviours/idmind_behaviours.h"

bool IdmindBehaviours::readFiles() //Change from spaces to tabs in files!!!
{
  std::vector<std::vector<int> > my_c;
  readNumbersFile("colours.txt", &my_c);

  for (int i=0; i<my_c.size(); i++)
  {
    Colour c;
    c.red = my_c[i][0];
    c.green = my_c[i][1];
    c.blue = my_c[i][2];

    colours_.push_back(c);
  }

  std::vector<std::vector<int> > my_b;
  readNumbersFile("behaviours.txt", &my_b);

  for (int i=0; i<my_b.size(); i++)
  {
    Behaviour b;
    b.head = my_b[i][0];
    b.left_arm = my_b[i][1];
    b.right_arm = my_b[i][2];
    b.mouth = my_b[i][3];

    for (int j=4; j<10; j++)
      b.leds.push_back(my_b[i][j]);

    behaviours_.push_back(b);
  }

  std::vector<std::vector<std::string> > my_m;
  readStringsFile("media.txt", &my_m);

  for (int i=0; i<my_m.size(); i++)
  {
    Media m;
    m.type = my_m[i][0];
    m.file_name = my_m[i][1];

    media_.push_back(m);
  }

  std::vector<std::vector<double> > my_p;
  readNumbersFile("points.txt", &my_p);

  for (int i=0; i<my_p.size(); i++)
  {
    Point p;
    p.x = my_p[i][0];
    p.y = my_p[i][1];
    p.th = my_p[i][2];
    p.stop = my_p[i][3];
    p.behaviour = my_p[i][4];
    p.media = my_p[i][5];

    points_.push_back(p);
  }

//  for (int d=0; d<my_vector.size(); d++)
//    for (int j=0; j<my_vector[d].size(); j++)
//      ROS_INFO("%d", my_vector[d][j]);

//  for (int d=0; d<my_strings.size(); d++)
//    for (int j=0; j<my_strings[d].size(); j++)
//      ROS_INFO("%s", my_strings[d][j].c_str());

  return true;
}

template<typename T>
bool IdmindBehaviours::readNumbersFile(std::string file, std::vector<std::vector<T> >* vector)
{
  std::string homedir = getenv("HOME");
  std::ifstream my_file((homedir + behaviours_folder_ + file).c_str());

  if (!my_file)
    ROS_ERROR("%s --> Failed to open %s file.", ros::this_node::getName().c_str(), file.c_str());

  std::string my_line;
  std::vector<T> row;

  int i = 0;
  while (std::getline(my_file, my_line))
  {
    if (my_line.size() != 0)
    {
      (*vector).push_back(row);
      std::vector<int> spaces;

      for (int k=0; k<my_line.size(); k++)
        if (my_line[k] == ' ')
          spaces.push_back(k);

      for (int j=0; j<spaces.size()-1; j++)
        (*vector)[i].push_back(atof(my_line.substr(spaces[j]+1, spaces[j+1]-spaces[j]).c_str()));

      (*vector)[i].push_back(atof(my_line.substr(spaces[spaces.size()-1]+1).c_str()));

      i++;
    }
  }

  my_file.close();
  return true;
}

bool IdmindBehaviours::readStringsFile(std::string file, std::vector<std::vector<std::string> >* vector)
{
  std::string homedir = getenv("HOME");
  std::ifstream my_file((homedir + behaviours_folder_ + file).c_str());

  if (!my_file)
    ROS_ERROR("%s --> Failed to open %s file.", ros::this_node::getName().c_str(), file.c_str());

  std::string my_line;
  std::vector<std::string> row;

  int i = 0;
  while (std::getline(my_file, my_line))
  {
    if (my_line.size() != 0)
    {
      (*vector).push_back(row);
      std::vector<int> spaces;

      for (int k=0; k<my_line.size(); k++)
        if (my_line[k] == ' ')
          spaces.push_back(k);

      for (int j=0; j<spaces.size()-1; j++)
        (*vector)[i].push_back(my_line.substr(spaces[j]+1, spaces[j+1]-spaces[j]));

      (*vector)[i].push_back(my_line.substr(spaces[spaces.size()-1]+1));

      i++;
    }
  }

  my_file.close();
  return true;
}
