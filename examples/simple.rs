use cluster_cone::*;

fn main() {
    let triangles = vec![
        Triangle::from_points(
            &glm::vec3(0.0, 1.0, 0.0),
            &glm::vec3(0.0, 0.0, 1.0),
            &glm::vec3(0.0, 0.0, 10.0),
        ),
        Triangle::from_points(
            &glm::vec3(0.0, 1.0, 0.0),
            &glm::vec3(0.0, 0.0, 1.0),
            &glm::vec3(1.0, 0.0, 0.0),
        ),
    ];

    let bounds = compute_cone(&dbg!(triangles));

    let bounds = dbg!(bounds);
}
