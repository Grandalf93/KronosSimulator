#include "tinyxml2.h"
#include "labelCreator.h"
#include <string>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <chrono>
#include <functional>
#include <map>



        
tinyxml2::XMLElement * labelCreator::InsertNewLabel (tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent  ,const char* element, const char* text){
    tinyxml2::XMLElement *pElement = doc.NewElement(element);
    pElement->SetText(text);
    parent->LinkEndChild(pElement);
    return pElement;
}

tinyxml2::XMLElement * labelCreator::InsertNewLabel (tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent  ,const char* element, const char* text, const char* nameAttribute, const char* attributeValue){
    tinyxml2::XMLElement *pElement = doc.NewElement(element);
    pElement->SetAttribute(nameAttribute, attributeValue);
    pElement->SetText(text);
    parent->LinkEndChild(pElement);
    return pElement;
}

tinyxml2::XMLElement *  labelCreator::InsertNewLabel (tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent  ,const char* element, const char* nameAttribute, const char* attributeValue){
    tinyxml2::XMLElement *pElement = doc.NewElement(element);
    pElement->SetAttribute(nameAttribute, attributeValue);
    parent->LinkEndChild(pElement);
    return pElement;
}

tinyxml2::XMLElement *  labelCreator::InsertNewLabel(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const char* element ){
    tinyxml2::XMLElement *pElement = doc.NewElement(element);
    parent->LinkEndChild(pElement);
    return pElement;
}


  std::string labelCreator::gen_random(const int len) {
    static const char alphanum[] =
        "0123456789"        
        "abcdefghijklmnopqrstuvwxyz";
    std::string tmp_s;
    tmp_s.reserve(len);

    for (int i = 0; i < len; ++i) {
        tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    
    return tmp_s;
}


std::string labelCreator::NewXmlModel(tinyxml2::XMLDocument &NewDoc, const char* name, std::vector<ignition::math::Vector3d> points){

    std::string objectName;      
    //auto nameSelected = std_msgs::msg::String();  
    objectName = std::string(name) + std::string("_") + labelCreator::gen_random(rand() % 10 + 5);
    
    tinyxml2::XMLElement *pRoot = NewDoc.NewElement("model");
    pRoot->SetAttribute("name", objectName.c_str());
    NewDoc.InsertFirstChild(pRoot); 

    tinyxml2::XMLElement *pElement = labelCreator::InsertNewLabel(NewDoc, pRoot, "static", "true");

    if (std::string(name) == "Oak_tree"){
      labelCreator::TreeBodyConstruction(NewDoc, pRoot, points);      
    }else{
      labelCreator::PolygonBodyConstruction(NewDoc, pRoot, name, points);        
    }

    //labelCreator::TreeBodyConstruction(NewDoc, pRoot, points);

    return objectName;
}



void labelCreator::IntermediateConstruction (tinyxml2::XMLDocument &NewDoc, tinyxml2::XMLElement *parent, const char* submeshValue, const char* nameValue){

tinyxml2::XMLElement *pGeometry = labelCreator::InsertNewLabel(NewDoc,parent, "geometry", "\n");
tinyxml2::XMLElement *pMesh = labelCreator::InsertNewLabel(NewDoc,pGeometry, "mesh", "\n");
tinyxml2::XMLElement *pUri1 = labelCreator::InsertNewLabel(NewDoc,pMesh, "uri", "model://oak_tree/meshes/oak_tree.dae");
tinyxml2::XMLElement *pSubmesh = labelCreator::InsertNewLabel(NewDoc,pMesh, "submesh", "\n"); 
tinyxml2::XMLElement *pName = labelCreator::InsertNewLabel(NewDoc,pSubmesh, "name", submeshValue); 
tinyxml2::XMLElement *pMaterial = labelCreator::InsertNewLabel(NewDoc,parent, "material", "\n"); 
tinyxml2::XMLElement *pScript = labelCreator::InsertNewLabel(NewDoc,pMaterial, "script", "\n"); 
tinyxml2::XMLElement *pUri2 = labelCreator::InsertNewLabel(NewDoc,pScript, "uri", "model://oak_tree/materials/scripts/");
tinyxml2::XMLElement *pUri3 = labelCreator::InsertNewLabel(NewDoc,pScript, "uri", "model://oak_tree/materials/textures/");
tinyxml2::XMLElement *pName1 = labelCreator::InsertNewLabel(NewDoc,pScript, "name", nameValue);  
}


void labelCreator::TreeBodyConstruction(tinyxml2::XMLDocument &NewDoc,tinyxml2::XMLElement *pRoot, std::vector<ignition::math::Vector3d> points){
    
    std::string pose;
    tinyxml2::XMLElement *pLink = labelCreator::InsertNewLabel(NewDoc, pRoot, "link", "\n", "name", "link");
    tinyxml2::XMLElement *pCollision = labelCreator::InsertNewLabel(NewDoc, pLink, "collision", "\n", "name", "collision");
    tinyxml2::XMLElement *pGeometry = labelCreator::InsertNewLabel(NewDoc, pCollision, "geometry", "\n");
    tinyxml2::XMLElement * pMesh = labelCreator::InsertNewLabel(NewDoc, pGeometry, "mesh", "\n");
    tinyxml2::XMLElement *pUri = labelCreator::InsertNewLabel(NewDoc, pMesh, "uri", "model://oak_tree/meshes/oak_tree.dae");
    tinyxml2::XMLElement *pVisual = labelCreator::InsertNewLabel(NewDoc, pLink, "visual", "\n", "name", "branch");
    labelCreator::IntermediateConstruction(NewDoc, pVisual, "Branch","OakTree/Branch");   
    tinyxml2::XMLElement *pVisual1 = labelCreator::InsertNewLabel(NewDoc,pLink, "visual", "\n", "name", "bark"); 
    labelCreator::IntermediateConstruction(NewDoc, pVisual1, "Bark", "OakTree/Bark");  
    pose = std::to_string(points.at(0).X()) +std::string(" ")+ std::to_string(points.at(0).Y()) + std::string(" 0") + std::string(" 0") + std::string(" 0") + std::string(" 0");     
    tinyxml2::XMLElement *pPose = labelCreator::InsertNewLabel(NewDoc,pRoot, "pose", pose.c_str()); 

}


tinyxml2::XMLElement * labelCreator::ambientColor(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, std::string decision){

std::map<std::string, std::string> classesColors = {{"Sidewalk", "0.2 0.2 0.2 1.0"},
                                 {"Grass", "0.5 0.85 0.22 1"}, 
                                 {"Wall", "0.8 0.8 0.8 1"}, 
                                 {"Building","0 0 0.45 1"}};
auto ambientColor = classesColors.find(decision);
tinyxml2::XMLElement *ambient = labelCreator::InsertNewLabel(doc,parent, "ambient", (ambientColor->second).c_str());

 }


void labelCreator::PointIterator(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *elementPine, const char* entity, std::vector<ignition::math::Vector3d> points){
    std::map<std::string, std::string> classesHeight = {{"Sidewalk", "0.15"},
                                 {"Grass", "0.12"}, 
                                 {"Wall", "0.65"}, 
                                 {"Building","5"}};

    auto height = classesHeight.find(entity);                          
    
    int iterator = 0;
    std::string coordinates;
    for (auto &point : points) {
      if (iterator == 0) {     

        tinyxml2::XMLElement *pHeight = labelCreator::InsertNewLabel(xmlDoc, elementPine, "height",(height->second).c_str());          
        coordinates = std::to_string(point.X())+std::string(" ")+std::to_string(point.Y());     
        tinyxml2::XMLElement *pListElement = labelCreator::InsertNewLabel(xmlDoc, elementPine, "point", coordinates.c_str());    
        

      } else if (iterator == points.size() - 1) {
        coordinates = std::to_string(point.X())+std::string(" ")+std::to_string(point.Y()); 
     
        tinyxml2::XMLElement *pListElement = labelCreator::InsertNewLabel(xmlDoc, elementPine, "point", coordinates.c_str());
        coordinates = std::to_string(points.at(0).X())+std::string(" ")+std::to_string(points.at(0).Y());  
        tinyxml2::XMLElement *pListElementf = labelCreator::InsertNewLabel(xmlDoc, elementPine, "point", coordinates.c_str());             

      } else {      
          
        coordinates = std::to_string(point.X())+std::string(" ")+std::to_string(point.Y());
        tinyxml2::XMLElement *pListElement = labelCreator::InsertNewLabel(xmlDoc, elementPine, "point", coordinates.c_str());  
       
      }
      iterator++;
    }
  }


 void labelCreator::PolygonBodyConstruction(tinyxml2::XMLDocument &NewDoc,tinyxml2::XMLElement *pRoot, const char* entity, std::vector<ignition::math::Vector3d> points){
    tinyxml2::XMLElement *pElement = labelCreator::InsertNewLabel(NewDoc,pRoot, "pose", "0 0 0 0 0 0");
    tinyxml2::XMLElement *pLink = labelCreator::InsertNewLabel(NewDoc,pRoot, "link", "\n", "name", "link");
    tinyxml2::XMLElement *pVisual = labelCreator::InsertNewLabel(NewDoc,pLink, "visual", "\n", "name", "visual");
    tinyxml2::XMLElement *pGeometry = labelCreator::InsertNewLabel(NewDoc,pVisual, "geometry", "\n");
    tinyxml2::XMLElement *pLine = labelCreator::InsertNewLabel(NewDoc,pGeometry, "polyline", "\n");
    labelCreator::PointIterator(NewDoc, pLine, entity, points);
    tinyxml2::XMLElement *pMaterial= labelCreator::InsertNewLabel(NewDoc,pVisual, "material");
    tinyxml2::XMLElement * pAmbient = ambientColor(NewDoc, pMaterial, entity);
    tinyxml2::XMLElement *pCollision= labelCreator::InsertNewLabel(NewDoc,pLink, "collision", "name", "collision");  
    tinyxml2::XMLElement *pGeometry1 = labelCreator::InsertNewLabel(NewDoc,pCollision, "geometry");  
    tinyxml2::XMLElement *pLine1 = labelCreator::InsertNewLabel(NewDoc,pGeometry1, "polyline", "\n");
    labelCreator::PointIterator(NewDoc, pLine1, entity, points);
    tinyxml2::XMLElement *pMaterial1= labelCreator::InsertNewLabel(NewDoc,pCollision, "material");
    tinyxml2::XMLElement * pAmbient1 = ambientColor(NewDoc, pMaterial1, entity);
 }

