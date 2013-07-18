ccflags-y := -Ofast -ffast-math -fgcse-lm -fgcse-sm -fsched-spec-load -fforce-addr -fsingle-precision-constant -mcpu=cortex-a9 -mtune=cortex-a9 -marm -mfpu=neon -ftree-vectorize -mvectorize-with-neon-quad
obj-$(CONFIG_ION) +=	ion.o ion_heap.o ion_system_heap.o ion_carveout_heap.o
obj-$(CONFIG_ION_IOMMU)	+= ion_iommu_heap.o
obj-$(CONFIG_ION_TEGRA) += tegra/
