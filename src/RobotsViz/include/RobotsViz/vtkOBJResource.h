/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOBJResource.h

  Copyright (c) Nicola Piga
  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkOBJResource
 * @brief   read Wavefront .obj files
 *
 * vtkOBJResource is a source object that reads Wavefront .obj
 * files. The output of this source object is polygonal data.
 * @sa
 * vtkOBJImporter
*/

#ifndef vtkOBJResource_h
#define vtkOBJResource_h

#include "vtkIOGeometryModule.h" // For export macro
#include "vtkAbstractPolyDataReader.h"

namespace RobotsViz {
    class vtkOBJResource;
}

class VTKIOGEOMETRY_EXPORT RobotsViz::vtkOBJResource : public vtkAbstractPolyDataReader
{
public:
  static vtkOBJResource *New();
  vtkTypeMacro(vtkOBJResource,vtkAbstractPolyDataReader);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  void SetFileData(const std::string& data);

protected:
  vtkOBJResource();
  ~vtkOBJResource() override;

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;
private:
  bool read_raw_data(char* output, const int num_chars, FILE* file_stream, std::istringstream& data_stream);
  vtkOBJResource(const vtkOBJResource&) = delete;
  void operator=(const vtkOBJResource&) = delete;
  std::string data_;
  bool use_file_ = true;
};

#endif
