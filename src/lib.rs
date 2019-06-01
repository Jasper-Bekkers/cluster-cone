pub use nalgebra_glm as glm;

#[derive(Debug)]
pub struct Triangle {
    pub p0: glm::Vec3,
    pub p1: glm::Vec3,
    pub p2: glm::Vec3,
    pub normal: glm::Vec3,
}

impl Triangle {
    pub fn from_points(p0: &glm::Vec3, p1: &glm::Vec3, p2: &glm::Vec3) -> Triangle {
        Triangle {
            p0: *p0,
            p1: *p1,
            p2: *p2,
            normal: (p1 - p0).cross(&(p2 - p0)).normalize(),
        }
    }
}

#[derive(Debug)]
pub struct Bounds {
    pub center: glm::Vec3,
    pub radius: f32,
    pub cone_apex: glm::Vec3,
    pub cone_axis: glm::Vec3,
    pub cone_cutoff: f32,
}

pub fn compute_bounding_sphere(points: &[glm::Vec3]) -> (glm::Vec3, f32) {
    let mut pmin = [0; 3];
    let mut pmax = [0; 3];

    for (idx, p) in points.iter().enumerate() {
        for axis in 0..3 {
            if p[axis] < points[pmin[axis]][axis] {
                pmin[axis] = idx
            }

            if p[axis] > points[pmax[axis]][axis] {
                pmax[axis] = idx
            }
        }
    }

    let mut paxisd2 = 0.0;
    let mut paxis = 0;

    for axis in 0..3 {
        let p1 = points[pmin[axis]];
        let p2 = points[pmax[axis]];

        let d2 = glm::length2(&(p2 - p1));

        if d2 > paxisd2 {
            paxisd2 = d2;
            paxis = axis;
        }
    }

    let p1 = points[pmin[paxis]];
    let p2 = points[pmax[paxis]];

    let mut center = (p1 + p2) * 0.5;
    let mut radius = paxisd2.sqrt() * 0.5;

    for p in points.iter() {
        let d2 = glm::length2(&(p - center));

        if d2 > radius * radius {
            let d = d2.sqrt();

            let k = 0.5 + (radius / d) * 0.5;

            center = center * k + p * (1.0 - k);
            radius = (radius + d) * 0.5;
        }
    }

    (center, radius)
}

pub fn compute_cone(triangles: &[Triangle]) -> Bounds {
    let mut corners = vec![];
    let mut normals = vec![];

    for tri in triangles.iter() {
        corners.push(tri.p0);
        corners.push(tri.p1);
        corners.push(tri.p2);
        normals.push(tri.normal);
    }

    let point_sphere = compute_bounding_sphere(&corners);
    let normal_sphere = compute_bounding_sphere(&normals);

    let center = point_sphere.0;
    let axis = glm::normalize(&normal_sphere.0);

    let mut min_dot = 1.0;

    for n in normals.iter() {
        let dot = glm::dot(&n, &axis);
        if dot < min_dot {
            min_dot = dot;
        }
    }

    // degenerate cluster, normal cone is larger than a hemisphere => trivial accept
    // note that if mindp is positive but close to 0, the triangle intersection code below gets less stable
    // we arbitrarily decide that if a normal cone is ~168 degrees wide or more, the cone isn't useful
    if min_dot <= 0.1 {
        return Bounds {
            center,
            radius: point_sphere.1,
            cone_apex: glm::vec3(0.0, 0.0, 0.0),
            cone_axis: glm::vec3(0.0, 0.0, 0.0),
            cone_cutoff: 0.0,
        };
    }

    let mut maxt = 0.0;
    for tri in triangles.iter() {
        for p in [tri.p0, tri.p1, tri.p2].iter() {
            let c = center - p;
            let dc = glm::dot(&c, &tri.normal);
            let dn = glm::dot(&axis, &tri.normal);
            let t = dc / dn;

            if t > maxt {
                maxt = t;
            }
        }
    }

    Bounds {
        center,
        radius: point_sphere.1,
        cone_apex: center - axis * maxt,
        cone_axis: axis,
        cone_cutoff: (1.0 - min_dot * min_dot).sqrt(),
    }
}
