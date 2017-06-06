#ifndef O_TRANSFORM_H
#define O_TRANSFORM_H

#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>

class OTransform
{

public:
	explicit OTransform() {}

	explicit OTransform(const glm::quat& quat, const glm::vec3& origin = glm::vec3(0.f))
		: m_origin(origin)
	{
		m_basis = glm::mat3_cast(quat);
	}

	explicit OTransform(const glm::mat3& basis, const glm::vec3& origin = glm::vec3(0.f))
		: m_origin(origin), m_basis(basis)
	{}

	OTransform(const OTransform& other) : m_basis(other.m_basis), m_origin(other.m_origin)
	{}

	OTransform operator=(const OTransform& other)
	{
		m_basis = other.m_basis;
		m_origin = other.m_origin;
		return *this;
	}

	glm::vec3 operator()(const glm::vec3& x) const
	{
		return glm::vec3(glm::dot(x, m_basis[0]), glm::dot(x, m_basis[1]), glm::dot(x, m_basis[2])) + m_origin;
	}

	glm::vec3 operator*(const glm::vec3& x) const
	{
		return (*this)(x);
	}

	glm::quat operator*(const glm::quat& q) const
	{
		return GetRotation() * q;
	}

	OTransform& operator*=(const OTransform& t)
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
		return *this;
	}

	void Mult(const OTransform& t1, const OTransform& t2)
	{
		m_basis = t1.m_basis * t2.m_basis;
		m_origin = t1(t2.m_origin);
	}

	glm::mat3& GetBasis()
	{
		return m_basis;
	}

	const glm::mat3& GetBasis() const
	{
		return m_basis;
	}

	glm::vec3& GetOrigin()
	{
		return m_origin;
	}

	const glm::vec3& GetOrigin() const
	{
		return m_origin;
	}

	glm::quat GetRotation() const
	{
		return glm::quat_cast(m_basis);
	}

	glm::mat4 GetOpenGLMatrix() const
	{
		glm::mat4 result(1.f);
		result[0] = glm::vec4(m_basis[0], 0.f);
		result[1] = glm::vec4(m_basis[1], 0.f);
		result[2] = glm::vec4(m_basis[2], 0.f);
		result[3] = glm::vec4(m_origin, 1.f);
		return result;
	}

	void SetOrigin(const glm::vec3& origin)
	{
		m_origin = origin;
	}

	void SetBasis(const glm::mat3& basis)
	{
		m_basis = basis;
	}

	void SetRotation(const glm::quat& q)
	{
		m_basis = glm::mat3_cast(q);
	}

	void SetIdentity()
	{
		m_basis = glm::mat3(1.f);
		m_origin = glm::vec3(0.f);
	}

	OTransform Inverse() const
	{
		glm::mat3 inv = glm::transpose(m_basis);
		return OTransform(inv, inv * -m_origin);
	}

	OTransform operator*(const OTransform& t) const;

private:
	glm::mat3 m_basis;
	glm::vec3 m_origin;
};

inline OTransform OTransform::operator*(const OTransform& t) const
{
	return OTransform(m_basis * t.m_basis, (*this)(t.m_origin));
}

inline bool operator==(const OTransform& t1, const OTransform& t2)
{
	return (t1.GetBasis() == t2.GetBasis() &&
		t1.GetOrigin() == t2.GetOrigin());
}

#endif