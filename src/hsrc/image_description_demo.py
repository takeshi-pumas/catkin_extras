from transformers import VisionEncoderDecoderModel, ViTImageProcessor, AutoTokenizer, pipeline,LlavaForConditionalGeneration, AutoProcessor, AutoImageProcessor
import torch
from PIL import Image
# import requests
# model = VisionEncoderDecoderModel.from_pretrained("nlpconnect/vit-gpt2-image-captioning")
# feature_extractor = ViTImageProcessor.from_pretrained("nlpconnect/vit-gpt2-image-captioning")
# tokenizer = AutoTokenizer.from_pretrained("nlpconnect/vit-gpt2-image-captioning")

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# model_id="llava-hf/llava-1.5-7b-hf"
# model = LlavaForConditionalGeneration.from_pretrained(
#         model_id, 
#         torch_dtype=torch.float16, 

#     ).to(device)
# tokenizer = AutoTokenizer.from_pretrained("llava-hf/llava-1.5-7b-hf")
# image_processor = AutoImageProcessor.from_pretrained("llava-hf/llava-1.5-7b-hf")
# # device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# # model_id.to(device)
# mm_pipeline = pipeline("image-to-text",model, image_processor = image_processor, tokenizer = tokenizer)
# prompt = "<image>\nUSER: Please describe age, gender, and clothing of the person.\nASSISTANT:"
# # # url = "https://www.ilankelman.org/stopsigns/australia.jpg"
# # # image = Image.open(requests.get(url, stream=True).raw)
# # # captioner = pipeline(model="ydshieh/vit-gpt2-coco-en")
# image = '/home/robocanes/hsr_robocanes/src/robocup_hsr/scripts/behavior/speech/receptionist/guest_1.png'
# image = Image.open(image)
# output = mm_pipeline(image, prompt=prompt, generate_kwargs={"max_new_tokens": 200})
# # # print(mm_pipeline("/home/kasia/Downloads/person.jpg", "Please describe age, gender, and clothing of the person."))
# # print(output[0]["generated_text"])

# rest, n = output[0]["generated_text"].rsplit('ASSISTANT:', 2)
# print(n)
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# model.to(device)

# print(captioner("https://huggingface.co/spaces/llava-hf/llava-4bit/resolve/main/examples/baklava.png"))#, "How to make this pastry?")

def describe_person(image, desc, prompt):
    print("Describe person")
    torch.cuda.empty_cache()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model_id="llava-hf/llava-1.5-7b-hf"
    model = LlavaForConditionalGeneration.from_pretrained(
            model_id, 
            torch_dtype=torch.float16, 
            low_cpu_mem_usage=True,
            load_in_4bit=True

        )

    #tokenizer = AutoTokenizer.from_pretrained(model_id)
    tokenizer = AutoTokenizer.from_pretrained(model_id, use_fast=False)
    image_processor = AutoImageProcessor.from_pretrained(model_id)
    processor = AutoProcessor.from_pretrained(model_id)

    mm_pipeline = pipeline("image-to-text", model, image_processor = image_processor, tokenizer = tokenizer)
    # prompt = "<image>\nUSER: Please list gender, mood, posture, and clothing of the person. \nASSISTANT:"
    image = Image.open(image)
    inputs = processor(prompt, image, return_tensors='pt').to(0, torch.float16)
    # print(inputs)
    # output = model.generate(**inputs, generate_kwargs={"max_new_tokens": 200}, do_sample=False)
    output = model.generate(**inputs, max_new_tokens=200, do_sample=False)
    text = (processor.decode(output[0][2:], skip_special_tokens=True))
    # output = mm_pipeline(inputs, prompt=prompt, generate_kwargs={"max_new_tokens": 200})
    # print(output)
    rest, n = text.rsplit('ASSISTANT: ', 2)
    print(n)
    f_name = desc +"_person_description.txt"  
    f = open(f_name, "w")
    f.write(n)
    f.close()


# describe_person("/home/robocanes/hsr_robocanes/julio_pic.jpg", "guest_1", "<image>\nUSER: Tell me which of the cleaning items in the photo has the greatest weight. \nASSISTANT:")

# describe_person("/home/robocanes/hsr_robocanes/julio_pic.jpg", "guest_1", "<image>\nUSER: Please describe the items on the table. \nASSISTANT:")

describe_person("shoes_1.jpg", "guest_1", "<image>\nUSER: Is the person in the photo wearing shoes? \nASSISTANT:")

# with open("/home/robocanes/hsr_robocanes/guest_1_person_description.txt", "r") as file:
#     description = file.read()
#     prom = "<image>\nUSER: Does either person in the photo match the following description:" + description +" If so, where are they positioned. \nASSISTANT:"
#     describe_person("/home/robocanes/hsr_robocanes/receptionist/chair_2.jpg", "chair_2", prom)

# max_length = 40
# num_beams = 10
# gen_kwargs = {"max_length": max_length, "num_beams": num_beams}
# def predict_step(image_paths):
#   images = []
#   for image_path in image_paths:
#     i_image = Image.open(image_path)
#     if i_image.mode != "RGB":
#       i_image = i_image.convert(mode="RGB")

#     images.append(i_image)

#   pixel_values = feature_extractor(images=images, return_tensors="pt").pixel_values
#   pixel_values = pixel_values.to(device)

#   output_ids = model.generate(pixel_values, **gen_kwargs)

#   preds = tokenizer.batch_decode(output_ids, skip_special_tokens=True)
#   preds = [pred.strip() for pred in preds]
#   print("Done preds")
#   print(preds)
#   return preds


# predict_step(['/home/kasia/Downloads/person.jpg']) 
# tokenizer = AutoTokenizer.from_pretrained(pretrained_model_name_or_path=model_id)
# processor = AutoProcessor.from_pretrained(pretrained_model_name_or_path=model_id)

# model = LlavaForConditionalGeneration.from_pretrained(
#         pretrained_model_name_or_path=model_id, 
#         torch_dtype=torch.float16, 
#         low_cpu_mem_usage=True,
#         # tokenizer = tokenizer

#     )

# # tokenizer = AutoTokenizer.from_pretrained(pretrained_model_name_or_path=model_id)

# inputs = processor(text=prompt, images=image, return_tensors="pt").to(0, torch.float16)
# output = model.generate(**inputs, max_new_tokens=200, do_sample=False).to(0, torch.float16)
# mm_pipeline = pipeline("image-to-text",model, tokenizer)
# output = mm_pipeline(image, prompt=prompt, generate_kwargs={"max_new_tokens": 200})

# # Generate
# generate_ids = model.generate(**inputs, max_new_tokens=15)
# processor.batch_decode(generate_ids, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]