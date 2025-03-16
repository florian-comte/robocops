### Training a New Model Based on YOLO (Tested with v11n and v4s)

1. **Create a Dataset on Roboflow**  
   - Ensure the dataset includes a version within the project.

2. **Train the Model**  
   - Use the `train_yolov11_notebook` (tested on Google Colab).

3. **Convert the Model from `.pt` to `.blob`**  
   - First, attempt to use the tool at [tools.luxonis.com](https://tools.luxonis.com).  
   - If the conversion fails:  
     - Use the Docker container with Luxonis tools to generate an `.onnx` file.  
        - Follow these steps:
            - Place the `tools_folder` in your `luxonis/tools` folder (created from the clone). Accept the overwriting of docker_compose.yml
            - Build the Docker container (refer to the `luxonis/tools` README for details).
            - Run the following command to convert the model:
                ```bash
                docker compose run tools-cli ./shared_with_container/your_model.pt
                ```
                - Replace `your_model.pt` with the name of your model file, and ensure it's in the shared folder.
     - Convert the `.onnx` file to `.blob` using [blobconverter.luxonis.com](https://blobconverter.luxonis.com).  

4. **Store the Converted Model**  
   - Place the resulting `.blob` file in the `trained_models` folder.

5. **Verify the Model**  
   - Update the `test_model_live` script to verify the model works live.