#include "vec.hpp"
#include <vector>
#include <fstream>

constexpr int width = 1920;
constexpr int height = 1080;

struct Ray {
    // Ray properties
    Vec3 position;
    Vec3 direction;
};

struct Material {
    Vec3 color;
    Vec3 emissionColor;
    float emissionStrength;
    float metalness;
};

struct Sphere {
    Vec3 center;
    float radius;
    Material* material;
};

struct RayHit {
    Vec3 position;
    Vec3 normal;
    float distance;
    Material* material;
};

struct Plane {
    Vec3 position;
    Vec3 normal;
    Material* material;
};

struct Triangle {
    Vec3 v0;
    Vec3 v1;
    Vec3 v2;
};

struct Vertex {
    Vec3 position;
    Vec3 normal;
    Vec2 uv;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    Material* material;
};

Ray rayGen(Vec2 uv) { 
    Vec3 origin(0.0f, 0.0f, 0.0f);
    float aspectRatio = static_cast<float>(width) / static_cast<float>(height); 
    Vec2 tmp = Vec2(((uv.getX() / width) * 2.0f - 1.0f) * aspectRatio, ((uv.getY() / height) * 2.0f - 1.0f) * -1.0f);
    Vec3 direction = Vec3(tmp.getX(), tmp.getY(), -1.0f).normalize();
    return {origin, direction};
}

bool intersect(const Ray& ray, const Sphere& sphere, RayHit& hit) {
    Vec3 oc = ray.position - sphere.center;
    float a = ray.direction.dot(ray.direction);
    float b = 2.0f * oc.dot(ray.direction);
    float c = oc.dot(oc) - sphere.radius * sphere.radius;
    float discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        hit.distance = (-b - std::sqrt(discriminant)) / (2.0f * a);
        hit.position = ray.position + ray.direction * hit.distance;
        hit.normal = (hit.position - sphere.center).normalize();
        hit.material = sphere.material;

        return hit.distance > 0;
    }

    return false;
}

bool intersect(const Ray& ray, const Plane& plane, RayHit& hit) {
    float denom = plane.normal.dot(ray.direction);
    if (std::abs(denom) > 1e-6) {
        Vec3 p0l0 = plane.position - ray.position;
        float t  = p0l0.dot(plane.normal) / denom;
        if (t >= 0) {
            hit.distance = t;
            hit.position = ray.position + ray.direction * hit.distance;
            hit.normal = plane.normal;
            hit.material = plane.material;
            return true;
        }
    }
    return false;
}

bool intersect(const Ray& ray, const Triangle& triangle, RayHit& hit) {
    Vec3 edge1 = triangle.v1 - triangle.v0;
    Vec3 edge2 = triangle.v2 - triangle.v0;
    Vec3 h = ray.direction.cross(edge2);
    float a = edge1.dot(h);
    if (std::abs(a) < 1e-6)
        return false; // Ray is parallel to triangle

    float f = 1.0f / a;
    Vec3 s = ray.position - triangle.v0;
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f)
        return false;

    Vec3 q = s.cross(edge1);
    float v = f * ray.direction.dot(q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    // Compute t to find out where the intersection point is on the line
    float t = f * edge2.dot(q);
    if (t > 1e-6) { // Ray intersection
        hit.distance = t;
        hit.position = ray.position + ray.direction * hit.distance;
        hit.normal = edge1.cross(edge2).normalize();
        return true;
    }
    
    return false; // Line intersection but not a ray intersection
}

bool intersect(const Ray& ray, const Mesh& mesh, RayHit& hit) {
    bool didHit = false;
    hit.distance = 1e30f;
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        Triangle triangle = {
            mesh.vertices[mesh.indices[i]].position,
            mesh.vertices[mesh.indices[i + 1]].position,
            mesh.vertices[mesh.indices[i + 2]].position
        };
        RayHit tempHit;
        if (intersect(ray, triangle, tempHit) && tempHit.distance < hit.distance) {
            hit = tempHit;
            hit.material = mesh.material;
            didHit = true;
        }
    }
    return didHit;
}

class Scene {
    public:
        Scene() {
            Material material;
            material.color = Vec3(1.0f, .0f, .0f);
            material.emissionColor = Vec3(0.0f, 0.0f, 0.0f);
            material.emissionStrength = 0.0f;
            material.metalness = 1.0f;
            materials.push_back(material);

            material.color = Vec3(0.0f, 0.0f, 0.0f);
            material.emissionColor = Vec3(1.0f, 1.0f, 1.0f);
            material.emissionStrength = 1.0f;
            material.metalness = 0.0f;
            materials.push_back(material);

            material.color = Vec3(1.0f, 1.0f, 1.0f);
            material.emissionColor = Vec3(0.0f, 0.0f, 0.0f);
            material.emissionStrength = 0.0f;
            material.metalness = 0.0f;
            materials.push_back(material);

            // Initialize scene with objects
            Sphere sphere = {Vec3(-1.0f, 0.0f, -2.0f), 1.0f, &materials[0]};
            spheres.push_back(sphere);
            sphere = {Vec3(1.0f, 0.0f, -2.0f), 1.0f, &materials[0]};
            spheres.push_back(sphere);
            sphere = {Vec3(-4.0f, 5.0f, -10.0f), 5.0f, &materials[1]};
            spheres.push_back(sphere);

            Plane plane = {Vec3(0.0f, -1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), &materials[2]};
            planes.push_back(plane);

        }
        Vec3 trace(int x, int y) {
            Vec2 uv(static_cast<float>(x), static_cast<float>(y));
            Ray ray = rayGen(uv);    
            Vec3 result = Vec3(0.0f, 0.0f, 0.0f);
            Vec3 ray_color = Vec3(1.0f, 1.0f, 1.0f);
            
            for (int i = 0; i < maxBounces; i++) {
                RayHit closestHit;
                if (this->closestHit(ray, closestHit)) {
                    ray.position = closestHit.position + closestHit.normal * 0.001f; // Offset to avoid self-intersection
                    Vec3 diffuseDirection = Vec3::randomInHemisphere(closestHit.normal);
                    Vec3 specularDirection = ray.direction.reflect(closestHit.normal);
                    ray.direction = diffuseDirection.lerp(specularDirection, closestHit.material->metalness).normalize();

                    Vec3 emittedLight = closestHit.material->emissionColor * closestHit.material->emissionStrength;
                    result = result + (ray_color * emittedLight);
                    ray_color = ray_color * (closestHit.material->color * closestHit.normal.dot(ray.direction));

                  
                } else {
                    result = result + (ray_color * Vec3(0.5f, 0.7f, 1.0f)); // Background color
                    break;
                }
            }
            
            return result;
        }
    
    private: 
        bool closestHit(Ray ray, RayHit& outHit) {
            RayHit closestHit;
            closestHit.distance = 1e30f;
            bool didHit = false;

            for (const auto& sphere : spheres) {
                RayHit hit;
                if (intersect(ray, sphere, hit) && (hit.distance < closestHit.distance)) {
                    closestHit = hit;
                    didHit = true;
                }
            }
            for (const auto& plane : planes) {
                RayHit hit;
                if (intersect(ray, plane, hit) && (hit.distance < closestHit.distance)) {
                    closestHit = hit;
                    didHit = true;
                }
            }
            for (const auto& mesh : meshes) {
                RayHit hit;
                if (intersect(ray, mesh, hit) && (hit.distance < closestHit.distance)) {
                    closestHit = hit;
                    didHit = true;
                }
            }

            outHit = closestHit;
            return didHit;
        }

        std::vector<Sphere> spheres;
        std::vector<Material> materials;
        std::vector<Plane> planes;
        std::vector<Mesh> meshes;
        int maxBounces = 4;
};


int main() {
    Scene scene = Scene();
    std::ofstream output("output.ppm");
    output << "P3\n" << width << " " << height << "\n255\n";


    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Process pixel at (x, y)
            Vec3 pixel = scene.trace(x, y);
            int r = static_cast<int>(255.99f * pixel.getX());
            int g = static_cast<int>(255.99f * pixel.getY());
            int b = static_cast<int>(255.99f * pixel.getZ());
            output << r << " " << g << " " << b << "\n";
        }
    }

}