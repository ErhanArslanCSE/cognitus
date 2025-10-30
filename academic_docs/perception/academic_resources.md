# cognitus_perception - Academic Resources

Academic papers, research, and resources related to visual perception, object detection, spatial relations, and anomaly detection.

**Last Updated:** October 30, 2025

---

## Object Detection & 3D Localization

### 1. YOLOv10: Real-Time End-to-End Object Detection (2024)
- **Authors:** Ao Wang, Hui Chen, Lihao Liu et al. (Tsinghua University)
- **Publication:** arXiv 2024
- **Link:** https://arxiv.org/pdf/2405.14458
- **Key Innovation:** End-to-End head eliminates NMS, real-time performance
- **Relevance:** Used in perception_node.py for object detection
- **GitHub:** https://github.com/THU-MIG/yolov10

### 2. YOLOv9: Learning What You Want to Learn Using Programmable Gradient Information (2024)
- **Publication:** CVPR 2024
- **Key Innovation:** Programmable Gradient Information (PGI), GELAN architecture
- **Performance:** State-of-the-art accuracy with fewer parameters
- **Relevance:** Alternative to YOLOv8 for better accuracy

### 3. RGB-D Cube R-CNN: 3D Object Detection with Selective Modality Dropout (2024)
- **Authors:** Piekenbrinck et al.
- **Publication:** CVPR 2024 Workshop on MULA
- **Link:** https://openaccess.thecvf.com/content/CVPR2024W/MULA/papers/Piekenbrinck_RGB-D_Cube_R-CNN_3D_Object_Detection_with_Selective_Modality_Dropout_CVPRW_2024_paper.pdf
- **Key Innovation:** RGB-D fusion with selective modality dropout
- **Relevance:** 3D localization from RGB-D (exactly our use case)

### 4. A Survey of Deep Learning-Driven 3D Object Detection (2024)
- **Publication:** PMC 2024
- **Link:** https://pmc.ncbi.nlm.nih.gov/articles/PMC12196975/
- **Coverage:** Comprehensive survey, 2015-2024 papers
- **Topics:** Sensor modalities, architectures, applications
- **Relevance:** Overview of state-of-the-art methods

### 5. YOLO Series Comprehensive Review (2025)
- **Title:** "YOLO advances to its genesis: A decadal and comprehensive review"
- **Publication:** Artificial Intelligence Review 2025
- **Link:** https://link.springer.com/article/10.1007/s10462-025-11253-3
- **Coverage:** Complete YOLO family evolution
- **Relevance:** Understanding YOLO ecosystem

---

## Scene Graphs & Spatial Relations

### 6. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (2024)
- **Publication:** ICRA 2024
- **Link:** https://concept-graphs.github.io/
- **Key Innovation:** Open-vocabulary 3D scene graphs using foundation models
- **Features:** Multi-view association, semantic relationships
- **Relevance:** Scene graph generation architecture
- **Code:** Available on project website

### 7. Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems (2024)
- **Authors:** Nathan Hughes, Yun Chang, Luca Carlone et al. (MIT)
- **Publication:** IJRR 2024
- **Link:** https://journals.sagepub.com/doi/full/10.1177/02783649241229725
- **Key Contribution:** Hierarchical 3D spatial perception
- **Relevance:** Real-time spatial understanding

### 8. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs (2024)
- **Authors:** MIT SPARK Lab
- **Publication:** IEEE RA-L, Vol. 9, Iss. 10, pp. 8921-8928, 2024
- **Key Innovation:** Real-time open-set scene graph generation
- **Relevance:** Task-driven scene understanding

### 9. Task and Motion Planning in Hierarchical 3D Scene Graphs (2024)
- **Publication:** ISRR 2024
- **Relevance:** Using scene graphs for planning
- **Application:** Robotic manipulation and navigation

### 10. Review on Scene Graph Generation Methods (2024)
- **Authors:** Monesh S, Senthilkumar N C
- **Publication:** Sage Journals 2024
- **Link:** https://journals.sagepub.com/doi/10.3233/MGS-230132
- **Coverage:** Comprehensive review of scene graph methods
- **Relevance:** Survey of different approaches

---

## Anomaly Detection

### 11. Unsupervised Anomaly Detection for Improving Adversarial Robustness of 3D Object Detection Models (2025)
- **Publication:** Electronics 2025, Vol. 14, Iss. 2
- **Link:** https://www.mdpi.com/2079-9292/14/2/236
- **Key Innovation:** Unsupervised anomaly detection for 3D models
- **Relevance:** Exactly our anomaly_detector.py approach

### 12. Towards Zero-Shot Anomaly Detection and Reasoning with Multimodal Large Language Models (2025)
- **Publication:** CVPR 2025
- **Authors:** Xu et al.
- **Link:** https://openaccess.thecvf.com/content/CVPR2025/papers/Xu_Towards_Zero-Shot_Anomaly_Detection_and_Reasoning_with_Multimodal_Large_Language_CVPR_2025_paper.pdf
- **Key Innovation:** Zero-shot anomaly detection with LLMs
- **Relevance:** Future enhancement for anomaly reasoning

### 13. An Unsupervised Method for Industrial Image Anomaly Detection with Vision Transformer-Based Autoencoder (2024)
- **Publication:** PMC 2024
- **Link:** https://pmc.ncbi.nlm.nih.gov/articles/PMC11054379/
- **Approach:** ViT-based autoencoder for unsupervised detection
- **Relevance:** Alternative architecture for anomaly detection

### 14. A Systematic Survey: Role of Deep Learning-based Image Anomaly Detection in Industrial Inspection Contexts (2025)
- **Publication:** Frontiers in Robotics and AI 2025
- **Link:** https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1554196/full
- **Coverage:** Comprehensive survey of industrial anomaly detection
- **Relevance:** Understanding different approaches

---

## RGB-D Processing & 3D Vision

### 15. CG-SLAM: Efficient Dense RGB-D SLAM in a Consistent Uncertainty-aware 3D Gaussian Field (2024)
- **Publication:** ECCV 2024
- **Key Innovation:** Dense RGB-D SLAM with 3D Gaussian fields
- **Relevance:** RGB-D processing for mapping

---

## GitHub Repositories & Code Resources

### 16. Awesome Industrial Anomaly Detection
- **Link:** https://github.com/M-3LAB/awesome-industrial-anomaly-detection
- **Content:** Curated list of anomaly detection papers and datasets
- **Updated:** Continuously (2024-2025)
- **Relevance:** Comprehensive resource for anomaly detection

### 17. Ultralytics YOLOv8 Official Repository
- **Link:** https://github.com/ultralytics/ultralytics
- **Content:** Official implementation, documentation
- **Features:** Training, inference, export to multiple formats
- **Relevance:** Primary object detection library used

### 18. CVPR 2024 Papers Collection
- **Link:** https://github.com/52CV/CVPR-2024-Papers
- **Content:** All CVPR 2024 papers organized by topic
- **Topics:** Object detection, 3D vision, scene understanding
- **Relevance:** Latest computer vision research

### 19. ECCV 2024 Papers Collection
- **Link:** https://github.com/52CV/ECCV-2024-Papers
- **Content:** All ECCV 2024 papers with code
- **Relevance:** European computer vision conference papers

---

## Key Conferences & Venues

**Computer Vision:**
- CVPR (Computer Vision and Pattern Recognition)
- ICCV (International Conference on Computer Vision)
- ECCV (European Conference on Computer Vision)

**Robotics:**
- ICRA (International Conference on Robotics and Automation)
- IROS (International Conference on Intelligent Robots and Systems)
- RSS (Robotics: Science and Systems)

**Recent Papers:** https://arxiv.org/list/cs.CV/recent

---

## Research Trends (2024-2025)

**Object Detection:**
- YOLO series evolution (v8 → v10 → v11)
- End-to-end detection (no NMS)
- Efficiency improvements (fewer parameters, faster inference)

**3D Perception:**
- RGB-D fusion techniques
- 3D Gaussian fields for SLAM
- Open-vocabulary detection

**Scene Understanding:**
- Open-vocabulary scene graphs
- Foundation model integration
- Hierarchical spatial representations

**Anomaly Detection:**
- Zero-shot detection
- Unsupervised learning
- Vision transformer-based methods

---

## Relevant Workshops

**CVPR 2024:**
- MULA Workshop (Multi-Modal Learning for Autonomous Systems)

**ICRA 2024/2025:**
- 3D Scene Understanding workshops
- Perception for Mobile Robotics

**IROS 2024:**
- Vision-based Navigation and Perception

---

## Implementation Notes

**For cognitus_perception:**
- YOLOv8/v10 for object detection ✓
- RGB-D Cube R-CNN approach for 3D localization ✓
- Unsupervised anomaly detection ✓
- Scene graph generation ✓

**Recommended upgrades:**
- YOLOv10 (NMS-free) for speed
- ConceptGraphs approach for open-vocabulary
- ViT-based anomaly detection for better accuracy

---

**Curated by:** COGNITUS Team
**Focus Areas:** Object Detection, 3D Localization, Spatial Relations, Anomaly Detection
**Last Literature Review:** October 2025
