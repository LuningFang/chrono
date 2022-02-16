// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alesandro Tasora, Radu Serban
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTriangleMeshShape)

ChTriangleMeshShape::ChTriangleMeshShape()
    : name(""), scale(ChVector<>(1)), wireframe(false), backface_cull(false), fixed_connectivity(false) {
    trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
};

void ChTriangleMeshShape::SetMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh, bool load_materials) {
    trimesh = mesh;

    auto filename = mesh->m_filename;
    if (load_materials && !filename.empty()) {
        std::vector<tinyobj::shape_t> shapes;
        tinyobj::attrib_t att;
        std::vector<tinyobj::material_t> materials;
        std::string warn;
        std::string err;

        auto mtl_base = filesystem::path(filesystem::path(filename).parent_path()).str();
        bool success = tinyobj::LoadObj(&att, &shapes, &materials, &warn, &err, filename.c_str(), mtl_base.c_str());
        if (!success) {
            std::cerr << "Error loading OBJ file " << filename << std::endl;
            std::cerr << "   tiny_obj warning message: " << warn << std::endl;
            std::cerr << "   tiny_obj error message:   " << err << std::endl;
            std::cerr << "No materials loaded." << std::endl;
            return;
        }

        // copy in materials
        for (int i = 0; i < materials.size(); i++) {
            std::shared_ptr<ChVisualMaterial> mat = chrono_types::make_shared<ChVisualMaterial>();
            mat->SetAmbientColor({materials[i].ambient[0], materials[i].ambient[1], materials[i].ambient[2]});

            mat->SetDiffuseColor({materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]});
            mat->SetSpecularColor({materials[i].specular[0], materials[i].specular[1], materials[i].specular[2]});
            mat->SetMetallic(materials[i].metallic);
            mat->SetTransparency(materials[i].dissolve);
            mat->SetRoughness(materials[i].roughness);

            // If metallic and roughness is set to default, use specular workflow
            if (materials[i].metallic == 0 && materials[i].roughness == 0.5) {
                mat->SetUseSpecularWorkflow(true);
            } else {
                mat->SetUseSpecularWorkflow(false);
            }

            if (materials[i].diffuse_texname != "") {
                mat->SetKdTexture(mtl_base + "/" + materials[i].diffuse_texname);
            }

            if (materials[i].specular_texname != "") {
                mat->SetKsTexture(mtl_base + materials[i].specular_texname);
                mat->SetUseSpecularWorkflow(true);
            }
            // set normal map when called "bump_texname"
            if (materials[i].bump_texname != "") {
                mat->SetNormalMapTexture(mtl_base + materials[i].bump_texname);
            }
            // set normal map when called "normal_texname"
            if (materials[i].normal_texname != "") {
                mat->SetNormalMapTexture(mtl_base + materials[i].normal_texname);
            }
            // set roughness texture if it exists
            if (materials[i].roughness_texname != "") {
                mat->SetRoughnessTexture(mtl_base + materials[i].roughness_texname);
                mat->SetUseSpecularWorkflow(false);
            }
            // set metallic texture if it exists
            if (materials[i].metallic_texname != "") {
                mat->SetMetallicTexture(mtl_base + materials[i].metallic_texname);
            }
            // set opacity texture if it exists
            // NOTE: need to make sure alpha and diffuse names are different to prevent 4 channel opacity textures in
            // Chrono::Sensor
            if (materials[i].alpha_texname != "" && materials[i].alpha_texname != materials[i].diffuse_texname) {
                mat->SetOpacityTexture(mtl_base + materials[i].alpha_texname);
            }

            material_list.push_back(mat);
        }

        // for each shape, copy material_indices
        trimesh->m_face_col_indices.clear();

        for (int i = 0; i < shapes.size(); i++) {
            for (int j = 0; j < shapes[i].mesh.indices.size() / 3; j++) {
                if (shapes[i].mesh.material_ids[j] < 0 || shapes[i].mesh.material_ids[j] >= material_list.size()) {
                    trimesh->m_face_col_indices.push_back(ChVector<int>(0, 0, 0));
                } else {
                    trimesh->m_face_col_indices.push_back(ChVector<int>(shapes[i].mesh.material_ids[j], 0, 0));
                }
            }
        }

    
    }
}

void ChTriangleMeshShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTriangleMeshShape>();
    // serialize parent class
    ChVisualization::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(trimesh);
    marchive << CHNVP(wireframe);
    marchive << CHNVP(backface_cull);
    marchive << CHNVP(name);
    marchive << CHNVP(scale);
}

void ChTriangleMeshShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChTriangleMeshShape>();
    // deserialize parent class
    ChVisualization::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(trimesh);
    marchive >> CHNVP(wireframe);
    marchive >> CHNVP(backface_cull);
    marchive >> CHNVP(name);
    marchive >> CHNVP(scale);
}

}  // end namespace chrono
