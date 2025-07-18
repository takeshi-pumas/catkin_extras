from transformers import LlavaForConditionalGeneration, LlavaProcessor, AutoTokenizer, AutoImageProcessor, BitsAndBytesConfig
from PIL import Image
import torch

def describe_person(image_path, output_prefix, prompt):
    print("📝 Describing person...")
    torch.cuda.empty_cache()

    model_id = "llava-hf/llava-1.5-7b-hf"
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Cuantización 4-bit moderna y segura
    quant_config = BitsAndBytesConfig(load_in_4bit=True)

    # Cargar modelo y componentes
    model = LlavaForConditionalGeneration.from_pretrained(
        model_id,
        torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
        quantization_config=quant_config,
        low_cpu_mem_usage=True,
        device_map="auto"
    )

    tokenizer = AutoTokenizer.from_pretrained(model_id, use_fast=False)
    image_processor = AutoImageProcessor.from_pretrained(model_id)
    processor = LlavaProcessor(tokenizer=tokenizer, image_processor=image_processor)

    # Abrir imagen
    image = Image.open(image_path).convert("RGB")

    # Preparar entrada
    inputs = processor(text=prompt, images=image, return_tensors="pt").to(device, model.dtype)

    # Generar descripción
    output = model.generate(**inputs, max_new_tokens=200)
    description = processor.tokenizer.decode(output[0], skip_special_tokens=True)

    # Extraer respuesta limpia
    if "ASSISTANT:" in description:
        description = description.split("ASSISTANT:")[-1].strip()

    # Guardar en archivo
    output_file = f"{output_prefix}_person_description.txt"
    with open(output_file, "w") as f:
        f.write(description)

    print("✅ Done. Description saved to:", output_file)
    print("🗣️", description)


# Ejemplo de uso
describe_person(
    "IMG_3681.jpg",
    "guest_1",
    "<image>\nUSER: Which object in the photo is the tallest? \nASSISTANT:"
)
