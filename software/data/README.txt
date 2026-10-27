\data\aJPEGImages                       Original images and annotated XML files（原始图片和标注后的xml文件）  
\data\images  
\data\images\train                      Training set of original images（原始图片训练集）  
\data\images\val                        Validation set of original images（原始图片验证集）  
\data\labels                            Training and validation TXT files converted from dota_txt（dota_txt文件训练集和验证集 txt文件训练集和验证集）  
\data\val_dota                          Files generated from XML → dota_xml（xml --> dota_xml生成文件）  
\data\val_original                      Files generated from dota_xml → dota_txt（dota_xml --> dota_txt生成文件）  
\data\create_training_directories.py    Create folders required for training（创建训练所需文件夹）  
\data\dota_to_txt.py                    File conversion: XML → dota_xml → dota_txt（文件转换xml --> dota_xml --> dota_txt）  
\data\roxml_to_dota.py                  File conversion: dota_txt → TXT（文件转换dota_txt --> txt）  
\data\split_data.py                     Split \data\images and \data\val_original into 90% training set and 10% validation set（将\data\images和\data\val_original文件分90%作为训练集，10%为验证集）  
\data\create_folders.py                 Classify JPEGImages into folders（JPEGImages图片进行文件夹分类）  
\data\data.yaml                         YAML file required for training（训练需要调用的yaml文件）  
\data\images_360.py                     Rotate original images 360 degrees, generating one image every 2 degrees（将原始图片旋转360度，每隔2度生成1张）  
\data\trans_xml.py                      Rotate original XML annotations 360 degrees, generating one for every 2 degrees（将原始图片旋转360度，每隔2度生成1份）  
\data\yolov8_train.py                   Start training the yolov8-obb model（开始训练yolov8-obb模型）  
\data\yolov8n-obb.pt                    Weight file required for training（训练需要用到的权重文件）  
