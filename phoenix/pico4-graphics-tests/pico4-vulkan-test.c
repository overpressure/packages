// SPDX-License-Identifier: Unlicense
/* Render a full-screen triangle through Turnip and verify every readback pixel.
 * Headless GPU acceptance: this does not prove KMS presentation or panel output.
 * Run under an outer timeout; a broken kernel ioctl cannot be bounded here.
 */
#include <vulkan/vulkan.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIZE 256
#define CHECK(call) do { VkResult r_ = (call); if (r_ != VK_SUCCESS) { \
    fprintf(stderr, "%s: VkResult=%d\n", #call, r_); exit(1); } } while (0)

static void fail(const char *s) { fprintf(stderr, "%s\n", s); exit(1); }

static uint32_t memory_type(VkPhysicalDevice gpu, uint32_t bits, VkMemoryPropertyFlags flags) {
    VkPhysicalDeviceMemoryProperties p;
    vkGetPhysicalDeviceMemoryProperties(gpu, &p);
    for (uint32_t i = 0; i < p.memoryTypeCount; ++i)
        if ((bits & (1u << i)) && (p.memoryTypes[i].propertyFlags & flags) == flags) return i;
    fail("Required memory type absent"); return 0;
}

static VkShaderModule shader(VkDevice dev, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) fail("Cannot open test shader");
    if (fseek(f, 0, SEEK_END)) fail("Cannot seek test shader");
    long n = ftell(f);
    if (n <= 0 || n > 65536 || n % 4) fail("Invalid shader size");
    rewind(f);
    uint32_t *code = malloc((size_t)n);
    if (!code || fread(code, 1, (size_t)n, f) != (size_t)n) fail("Cannot read test shader");
    fclose(f);
    VkShaderModuleCreateInfo info = {.sType=VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, .codeSize=(size_t)n, .pCode=code};
    VkShaderModule result;
    CHECK(vkCreateShaderModule(dev, &info, NULL, &result)); free(code); return result;
}

int main(void) {
    VkApplicationInfo app = {.sType=VK_STRUCTURE_TYPE_APPLICATION_INFO, .pApplicationName="pico4-vulkan-test", .apiVersion=VK_API_VERSION_1_2};
    VkInstanceCreateInfo ici = {.sType=VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, .pApplicationInfo=&app};
    VkInstance instance; CHECK(vkCreateInstance(&ici, NULL, &instance));
    uint32_t count = 0;
    CHECK(vkEnumeratePhysicalDevices(instance, &count, NULL));
    if (count != 1) fail("Expected exactly one GPU; select the explicit Turnip ICD");
    VkPhysicalDevice gpu; CHECK(vkEnumeratePhysicalDevices(instance, &count, &gpu));
    VkPhysicalDeviceDriverProperties driver = {.sType=VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES};
    VkPhysicalDeviceProperties2 props = {.sType=VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2, .pNext=&driver};
    vkGetPhysicalDeviceProperties2(gpu, &props);
    fprintf(stderr, "GPU: %s; driver: %s (%s)\n", props.properties.deviceName, driver.driverName, driver.driverInfo);
    if (driver.driverID != VK_DRIVER_ID_MESA_TURNIP || props.properties.deviceType != VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU)
        fail("Hardware Turnip required; software rendering is not acceptance");
    uint32_t qcount=0;
    vkGetPhysicalDeviceQueueFamilyProperties(gpu, &qcount, NULL);
    VkQueueFamilyProperties *qp = calloc(qcount, sizeof(*qp));
    if (!qp) fail("Queue allocation failed");
    vkGetPhysicalDeviceQueueFamilyProperties(gpu, &qcount, qp);
    uint32_t family=UINT32_MAX;
    for (uint32_t i=0; i<qcount; ++i) if (qp[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) { family=i; break; }
    free(qp); if (family==UINT32_MAX) fail("No graphics queue");
    float priority=1;
    VkDeviceQueueCreateInfo qi={.sType=VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,.queueFamilyIndex=family,.queueCount=1,.pQueuePriorities=&priority};
    VkDeviceCreateInfo di={.sType=VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,.queueCreateInfoCount=1,.pQueueCreateInfos=&qi};
    VkDevice dev; CHECK(vkCreateDevice(gpu,&di,NULL,&dev));
    VkQueue queue; vkGetDeviceQueue(dev,family,0,&queue);

    VkImageCreateInfo ii={.sType=VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,.imageType=VK_IMAGE_TYPE_2D,.format=VK_FORMAT_R8G8B8A8_UNORM,
        .extent={SIZE,SIZE,1},.mipLevels=1,.arrayLayers=1,.samples=VK_SAMPLE_COUNT_1_BIT,.tiling=VK_IMAGE_TILING_OPTIMAL,
        .usage=VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT|VK_IMAGE_USAGE_TRANSFER_SRC_BIT,.sharingMode=VK_SHARING_MODE_EXCLUSIVE};
    VkImage image; CHECK(vkCreateImage(dev,&ii,NULL,&image));
    VkMemoryRequirements req; vkGetImageMemoryRequirements(dev,image,&req);
    VkMemoryAllocateInfo ai={.sType=VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,.allocationSize=req.size,.memoryTypeIndex=memory_type(gpu,req.memoryTypeBits,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)};
    VkDeviceMemory imem; CHECK(vkAllocateMemory(dev,&ai,NULL,&imem)); CHECK(vkBindImageMemory(dev,image,imem,0));
    VkImageViewCreateInfo vi={.sType=VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,.image=image,.viewType=VK_IMAGE_VIEW_TYPE_2D,.format=ii.format,
        .subresourceRange={VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1}};
    VkImageView view; CHECK(vkCreateImageView(dev,&vi,NULL,&view));
    VkBufferCreateInfo bi={.sType=VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,.size=SIZE*SIZE*4,.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT,.sharingMode=VK_SHARING_MODE_EXCLUSIVE};
    VkBuffer buffer; CHECK(vkCreateBuffer(dev,&bi,NULL,&buffer));
    vkGetBufferMemoryRequirements(dev,buffer,&req);
    ai.allocationSize=req.size; ai.memoryTypeIndex=memory_type(gpu,req.memoryTypeBits,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VkDeviceMemory bmem; CHECK(vkAllocateMemory(dev,&ai,NULL,&bmem)); CHECK(vkBindBufferMemory(dev,buffer,bmem,0));

    VkAttachmentDescription attachment={.format=ii.format,.samples=VK_SAMPLE_COUNT_1_BIT,.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR,
        .storeOp=VK_ATTACHMENT_STORE_OP_STORE,.stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE,.stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE,
        .initialLayout=VK_IMAGE_LAYOUT_UNDEFINED,.finalLayout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL};
    VkAttachmentReference ref={0,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription sub={.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS,.colorAttachmentCount=1,.pColorAttachments=&ref};
    VkSubpassDependency dep={.srcSubpass=0,.dstSubpass=VK_SUBPASS_EXTERNAL,.srcStageMask=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
        .dstStageMask=VK_PIPELINE_STAGE_TRANSFER_BIT,.srcAccessMask=VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,.dstAccessMask=VK_ACCESS_TRANSFER_READ_BIT};
    VkRenderPassCreateInfo ri={.sType=VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,.attachmentCount=1,.pAttachments=&attachment,
        .subpassCount=1,.pSubpasses=&sub,.dependencyCount=1,.pDependencies=&dep};
    VkRenderPass pass; CHECK(vkCreateRenderPass(dev,&ri,NULL,&pass));
    VkFramebufferCreateInfo fi={.sType=VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO,.renderPass=pass,.attachmentCount=1,.pAttachments=&view,.width=SIZE,.height=SIZE,.layers=1};
    VkFramebuffer fb; CHECK(vkCreateFramebuffer(dev,&fi,NULL,&fb));
    VkShaderModule vs=shader(dev,"/usr/share/pico4/graphics/fullscreen.vert.spv"), fs=shader(dev,"/usr/share/pico4/graphics/quadrants.frag.spv");
    VkPipelineShaderStageCreateInfo stages[2]={
        {.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,.stage=VK_SHADER_STAGE_VERTEX_BIT,.module=vs,.pName="main"},
        {.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,.stage=VK_SHADER_STAGE_FRAGMENT_BIT,.module=fs,.pName="main"}};
    VkPipelineVertexInputStateCreateInfo vertex={.sType=VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo assembly={.sType=VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST};
    VkViewport viewport={0,0,SIZE,SIZE,0,1}; VkRect2D scissor={{0,0},{SIZE,SIZE}};
    VkPipelineViewportStateCreateInfo vp={.sType=VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,.viewportCount=1,.pViewports=&viewport,.scissorCount=1,.pScissors=&scissor};
    VkPipelineRasterizationStateCreateInfo raster={.sType=VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,.polygonMode=VK_POLYGON_MODE_FILL,.cullMode=VK_CULL_MODE_NONE,.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE,.lineWidth=1};
    VkPipelineMultisampleStateCreateInfo ms={.sType=VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT};
    VkPipelineColorBlendAttachmentState blend={.colorWriteMask=15};
    VkPipelineColorBlendStateCreateInfo cb={.sType=VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,.attachmentCount=1,.pAttachments=&blend};
    VkPipelineLayoutCreateInfo li={.sType=VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    VkPipelineLayout layout; CHECK(vkCreatePipelineLayout(dev,&li,NULL,&layout));
    VkGraphicsPipelineCreateInfo pi={.sType=VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,.stageCount=2,.pStages=stages,.pVertexInputState=&vertex,
        .pInputAssemblyState=&assembly,.pViewportState=&vp,.pRasterizationState=&raster,.pMultisampleState=&ms,.pColorBlendState=&cb,.layout=layout,.renderPass=pass};
    VkPipeline pipeline; CHECK(vkCreateGraphicsPipelines(dev,VK_NULL_HANDLE,1,&pi,NULL,&pipeline));

    VkCommandPoolCreateInfo pci={.sType=VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,.queueFamilyIndex=family};
    VkCommandPool pool; CHECK(vkCreateCommandPool(dev,&pci,NULL,&pool));
    VkCommandBufferAllocateInfo cai={.sType=VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,.commandPool=pool,.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY,.commandBufferCount=1};
    VkCommandBuffer cmd; CHECK(vkAllocateCommandBuffers(dev,&cai,&cmd));
    VkCommandBufferBeginInfo cbi={.sType=VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT};
    CHECK(vkBeginCommandBuffer(cmd,&cbi));
    VkClearValue clear={.color={{0,0,0,0}}};
    VkRenderPassBeginInfo begin={.sType=VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,.renderPass=pass,.framebuffer=fb,.renderArea={{0,0},{SIZE,SIZE}},.clearValueCount=1,.pClearValues=&clear};
    vkCmdBeginRenderPass(cmd,&begin,VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline); vkCmdDraw(cmd,3,1,0,0); vkCmdEndRenderPass(cmd);
    VkBufferImageCopy copy={.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1},.imageExtent={SIZE,SIZE,1}};
    vkCmdCopyImageToBuffer(cmd,image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,buffer,1,&copy);
    VkMemoryBarrier barrier={.sType=VK_STRUCTURE_TYPE_MEMORY_BARRIER,.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT,.dstAccessMask=VK_ACCESS_HOST_READ_BIT};
    vkCmdPipelineBarrier(cmd,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&barrier,0,NULL,0,NULL);
    CHECK(vkEndCommandBuffer(cmd));
    VkFenceCreateInfo fence_info={.sType=VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; VkFence fence;
    CHECK(vkCreateFence(dev,&fence_info,NULL,&fence));
    VkSubmitInfo submit={.sType=VK_STRUCTURE_TYPE_SUBMIT_INFO,.commandBufferCount=1,.pCommandBuffers=&cmd};
    CHECK(vkQueueSubmit(queue,1,&submit,fence)); CHECK(vkWaitForFences(dev,1,&fence,VK_TRUE,10000000000ull));
    uint8_t *pixels; CHECK(vkMapMemory(dev,bmem,0,VK_WHOLE_SIZE,0,(void **)&pixels));
    unsigned bad=0;
    for (unsigned y=0;y<SIZE;++y) for (unsigned x=0;x<SIZE;++x) {
        uint8_t *p=pixels+4*(y*SIZE+x); int green=y<128?64:191;
        if (p[0]!=(x<128?255:0) || abs((int)p[1]-green)>1 || p[2]!=(x<128?0:255) || p[3]!=255) ++bad;
    }
    FILE *out=fopen("/run/pico4/vulkan-readback.rgba","wb");
    if (!out || fwrite(pixels,1,SIZE*SIZE*4,out)!=SIZE*SIZE*4 || fclose(out)) fail("Cannot save render readback");
    vkUnmapMemory(dev,bmem);
    printf("{\"status\":\"%s\",\"driver\":\"turnip\",\"device_id\":%u,\"api_version\":%u,\"width\":256,\"height\":256,\"verified_pixels\":%u,\"bad_pixels\":%u}\n",
        bad?"failed":"passed",props.properties.deviceID,props.properties.apiVersion,SIZE*SIZE-bad,bad);
    vkDestroyFence(dev,fence,NULL); vkDestroyCommandPool(dev,pool,NULL); vkDestroyPipeline(dev,pipeline,NULL);
    vkDestroyPipelineLayout(dev,layout,NULL); vkDestroyShaderModule(dev,vs,NULL); vkDestroyShaderModule(dev,fs,NULL);
    vkDestroyFramebuffer(dev,fb,NULL); vkDestroyRenderPass(dev,pass,NULL); vkDestroyImageView(dev,view,NULL);
    vkDestroyBuffer(dev,buffer,NULL); vkFreeMemory(dev,bmem,NULL); vkDestroyImage(dev,image,NULL); vkFreeMemory(dev,imem,NULL);
    vkDestroyDevice(dev,NULL); vkDestroyInstance(instance,NULL); return bad?1:0;
}
