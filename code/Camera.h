#include <glm\glm.hpp>

class Camera
{

public:
  
  
  
private:
  
	glm::vec3 m_position; // Current position
	glm::vec3 m_right; // Vector pointing to the right
	glm::vec3 m_up; // Vector pointing up
	glm::vec3 m_direction; // Direction camera is looking
};
