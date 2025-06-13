### How we trained our model (tested with yolov11n model)

1. **Create a Dataset on Roboflow**  
   - Ensure the dataset includes a version within the project. Feel free to use the dataset we created based on the previous years one with some data augmentation: https://app.roboflow.com/robocops/duplo-merged-v3/2

2. **Train the Model**  
   - Use the `train_yolov11_notebook` (tested on Google Colab).

3. **Convert the Model from `.pt` to `.blob`**  
   - Use the tool at [tools.luxonis.com](https://tools.luxonis.com).   

4. **Store the Converted Model**  
   - Place the resulting `.blob` file in the `trained_models` folder.

5. **Verify the Model**  
   - Update the `test_model_live` script to verify the model works live.