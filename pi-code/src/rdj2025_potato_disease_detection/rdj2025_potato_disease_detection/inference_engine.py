# import torch
# import torch.nn as nn
# from torchvision import models, transforms
# from PIL import Image


# class PotatoDiseaseModel:
#     def __init__(self):
#         self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        
#         # Load ResNet18 and modify final layer
#         self.model = models.resnet18(pretrained=False) ## Should also test with pretrained=True
#         num_ftrs = self.model.fc.in_features
#         self.model.fc = nn.Linear(num_ftrs, 3)  # 3 classes
#         model_path = './src/rdj2025_potato_disease_detection/models/model_ft_gpu.pth'
#         self.model.load_state_dict(torch.load(model_path, map_location="cpu"))
#         self.model = self.model.to(self.device)
#         self.model.eval()

#         self.class_names = ['Early_blight', 'Late_blight', 'Healthy']

#         # Define preprocessing
#         self.transform = transforms.Compose([
#             transforms.Resize(256),
#             transforms.CenterCrop(224),
#             transforms.ToTensor(),
#             transforms.Normalize([0.485, 0.456, 0.406],
#                                  [0.229, 0.224, 0.225])
#         ])

#     def predict(self, image: Image.Image) -> str:
#         """Run inference on a PIL image and return class name."""
#         with torch.no_grad():
#             x = self.transform(image).unsqueeze(0).to(self.device)
#             outputs = self.model(x)
#             _, pred_idx = torch.max(outputs, 1)
#             return self.class_names[pred_idx.item()]




"""
Inference Engine for Potato Disease Detection
Matches the training architecture from Google Colab
"""

import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
import os


class PotatoDiseaseModel:
    """
    Potato Disease Classification Model
    Architecture: ResNet18 with 3 output classes
    """
    
    def __init__(self, model_path=None):
        """
        Initialize the model for inference
        
        Args:
            model_path: Path to the trained model weights (.pth file)
                       If None, uses default path
        """
        # Device configuration
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        # Class names - MUST match training order
        self.class_names = ['Early_blight', 'Healthy', 'Late_blight']
        
        # Build model architecture (same as training)
        self.model = self._build_model()
        
        # Load trained weights
        if model_path is None:
            model_path = './src/rdj2025_potato_disease_detection/models/potato_disease_model.pth'
        
        self._load_weights(model_path)
        
        # Set to evaluation mode
        self.model = self.model.to(self.device)
        self.model.eval()
        
        # Define preprocessing (same as validation transforms in training)
        self.transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])
        
        print("Model loaded successfully!")
        print(f"Classes: {self.class_names}")
    
    def _build_model(self):
        """
        Build ResNet18 architecture
        MUST match training architecture exactly
        """
        # Load ResNet18 backbone
        model = models.resnet18(pretrained=False)
        
        # Modify final fully connected layer for 3 classes
        num_ftrs = model.fc.in_features  # 512 for ResNet18
        model.fc = nn.Linear(num_ftrs, 3)
        
        return model
    
    def _load_weights(self, model_path):
        """
        Load trained weights with proper handling
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model weights not found at: {model_path}")
        
        print(f"Loading model weights from: {model_path}")
        
        # Load state dict
        state_dict = torch.load(model_path, map_location="cpu")
        
        # Handle potential 'resnet.' prefix in keys
        new_state_dict = {}
        for k, v in state_dict.items():
            # Remove 'resnet.' prefix if present
            new_key = k.replace("resnet.", "") if k.startswith("resnet.") else k
            new_state_dict[new_key] = v
        
        # Load weights into model
        missing_keys, unexpected_keys = self.model.load_state_dict(new_state_dict, strict=False)
        
        if missing_keys:
            print(f"Warning - Missing keys: {missing_keys}")
        if unexpected_keys:
            print(f"Warning - Unexpected keys: {unexpected_keys}")
        
        if not missing_keys and not unexpected_keys:
            print("✓ All weights loaded successfully")
    
    def predict(self, image: Image.Image) -> str:
        """
        Run inference on a PIL image and return predicted class name
        
        Args:
            image: PIL Image object (RGB)
            
        Returns:
            str: Predicted class name ('Early_blight', 'Healthy', or 'Late_blight')
        """
        # Ensure image is RGB
        if image.mode != 'RGB':
            image = image.convert('RGB')
        
        # Preprocess image
        input_tensor = self.transform(image)
        input_batch = input_tensor.unsqueeze(0)  # Create batch dimension
        
        # Move to device
        input_batch = input_batch.to(self.device)
        
        # Inference
        with torch.no_grad():
            outputs = self.model(input_batch)
            _, predicted_idx = torch.max(outputs, 1)
        
        # Get class name
        class_name = self.class_names[predicted_idx.item()]
        
        return class_name
    
    def predict_with_confidence(self, image: Image.Image) -> tuple:
        """
        Run inference and return prediction with confidence scores
        
        Args:
            image: PIL Image object (RGB)
            
        Returns:
            tuple: (predicted_class_name, confidence_dict)
                  confidence_dict maps class names to confidence scores
        """
        # Ensure image is RGB
        if image.mode != 'RGB':
            image = image.convert('RGB')
        
        # Preprocess image
        input_tensor = self.transform(image)
        input_batch = input_tensor.unsqueeze(0)
        input_batch = input_batch.to(self.device)
        
        # Inference
        with torch.no_grad():
            outputs = self.model(input_batch)
            probabilities = torch.nn.functional.softmax(outputs, dim=1)
            confidence, predicted_idx = torch.max(probabilities, 1)
        
        # Get class name and all confidences
        predicted_class = self.class_names[predicted_idx.item()]
        confidence_dict = {
            class_name: probabilities[0][i].item() 
            for i, class_name in enumerate(self.class_names)
        }
        
        return predicted_class, confidence_dict
    
    def predict_batch(self, images: list) -> list:
        """
        Run inference on multiple images
        
        Args:
            images: List of PIL Image objects
            
        Returns:
            list: List of predicted class names
        """
        predictions = []
        
        for image in images:
            pred = self.predict(image)
            predictions.append(pred)
        
        return predictions


# For backward compatibility with old inference interface
def run_inference(pil_image) -> str:
    """
    Legacy function for compatibility
    Creates a model instance and runs prediction
    """
    model = PotatoDiseaseModel()
    result = model.predict(pil_image)
    return result


if __name__ == "__main__":
    """
    Test the inference engine
    """
    print("Testing Potato Disease Inference Engine")
    print("=" * 60)
    
    # Test with a sample image
    model = PotatoDiseaseModel()
    
    # Create a dummy test image
    test_image = Image.new('RGB', (224, 224), color='green')
    
    # Test basic prediction
    prediction = model.predict(test_image)
    print(f"\nBasic Prediction: {prediction}")
    
    # Test prediction with confidence
    pred_class, confidences = model.predict_with_confidence(test_image)
    print(f"\nPrediction with Confidence:")
    print(f"Predicted Class: {pred_class}")
    print("Confidence Scores:")
    for class_name, conf in confidences.items():
        print(f"  {class_name}: {conf*100:.2f}%")
    
    print("\n✓ Inference engine test complete!")